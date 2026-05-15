#!/usr/bin/python3

# eFinder_tetra3rs_mp — electronic finder scope, plate-solving over LX200/WiFi
# Derived from original work Copyright (C) 2025 Keith Venables (GPL v3)
# Simplified: direct picamera2, no Nexus, no GPIO, no LED, no WiFi switching
#
# Merged edition — combines the multiprocess architecture of the tiny_img
# branch with the tetra3rs Rust solver of the tetra3rs branch. Result:
#   - No cedar-detect gRPC server process
#   - No cedar-solve Python dependency
#   - In-process Rust centroid extraction + plate solve via tetra3rs
#   - Preserves tiny_img's mount-push, seeded-solve with blind fallback,
#     proc_specs-based restart pattern, and OTA update infrastructure.
#
#   Process 0 (main):    spawns worker processes; monitors health.
#   Process 1 (camera):  picamera2 capture loop -> shared memory frame slot.
#   Process 2 (solver):  tetra3rs extract + solve -> shared Values + JSON.
#   Process 3 (lx200):   LX200/WiFi server — reads Values directly, no IPC lag.
#
# Inter-process communication:
#   frame_shm      — SharedMemory  (760x960 uint8, camera -> solver)
#   frame_ready    — Event         (camera signals solver: new frame ready)
#   shared_ra      — Value(c_double) solver writes, lx200 reads — zero-copy
#   shared_dec     — Value(c_double) solver writes, lx200 reads — zero-copy
#   offset_flag    — Value(c_bool)   lx200/solver coordinate during offset meas.
#   test_mode      — Value(c_bool)   lx200 sets, camera reads
#   cmd_q          — Queue  lx200 -> solver: tuning/on-demand commands
#   result_q       — Queue  solver -> lx200: command results
#   cam_cmd_q      — Queue  solver -> camera: set_exp, capture_now, etc.
#   cam_result_q   — Queue  camera -> solver: capture results
#   maint_q        — Queue  maint server -> solver: maint commands
#   maint_res_q    — Queue  solver -> maint: maint responses
#

import csv
import io
import json
import logging
import math
import os
import re
import socket
import struct
import sys
import threading
import time
from ctypes import c_bool, c_double
from multiprocessing import (
    Event, Process, Queue, Value, set_start_method,
    shared_memory,
)
from pathlib import Path
from typing import Optional

import numpy as np
from PIL import Image, ImageEnhance

try:
    from picamera2 import Picamera2
except ImportError:
    Picamera2 = None

try:
    import tetra3
except ImportError:
    tetra3 = None

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s [%(name)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("efinder")

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
FRAME_H     = 760
FRAME_W     = 960
FRAME_BYTES = FRAME_H * FRAME_W          # uint8 grey
CENTRE_X    = FRAME_W / 2.0
CENTRE_Y    = FRAME_H / 2.0

SHM_NAME    = "efinder_frame"
LIVE_JPG    = "/dev/shm/efinder_live.jpg"
SOCKET_PATH = "/run/efinder/maint.sock"

DEFAULT_CONFIG = "/home/efinder/Solver/eFinder.config"
STARS_CSV      = "/home/efinder/Solver/NamedStars.csv"

# LX200 / WiFi server
LX200_HOST = "0.0.0.0"
LX200_PORT = 4030

# ---------------------------------------------------------------------------
# Config helpers
# ---------------------------------------------------------------------------

def load_config(path: str = DEFAULT_CONFIG) -> dict:
    cfg: dict = {}
    try:
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                if "=" in line:
                    k, _, v = line.partition("=")
                    cfg[k.strip()] = v.strip()
    except FileNotFoundError:
        log.warning("Config file not found: %s", path)
    return cfg


def save_config(cfg: dict, path: str = DEFAULT_CONFIG) -> None:
    lines = []
    try:
        with open(path) as f:
            lines = f.readlines()
    except FileNotFoundError:
        pass

    updated = set()
    new_lines = []
    for line in lines:
        stripped = line.strip()
        if not stripped or stripped.startswith("#") or "=" not in stripped:
            new_lines.append(line)
            continue
        k = stripped.split("=", 1)[0].strip()
        if k in cfg:
            new_lines.append(f"{k} = {cfg[k]}\n")
            updated.add(k)
        else:
            new_lines.append(line)
    for k, v in cfg.items():
        if k not in updated:
            new_lines.append(f"{k} = {v}\n")
    with open(path, "w") as f:
        f.writelines(new_lines)


# ---------------------------------------------------------------------------
# Named-star lookup (O(1) dict instead of O(N) CSV re-scan each solve)
# ---------------------------------------------------------------------------

def _build_starnames(csv_path: str) -> dict:
    """Return {name_lower: (ra_deg, dec_deg)} from NamedStars.csv."""
    stars: dict = {}
    try:
        with open(csv_path, newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                name = (row.get("name") or row.get("Name") or "").strip().lower()
                try:
                    ra  = float(row.get("ra")  or row.get("RA")  or "")
                    dec = float(row.get("dec") or row.get("Dec") or "")
                except (ValueError, TypeError):
                    continue
                if name:
                    stars[name] = (ra, dec)
    except FileNotFoundError:
        pass
    return stars


_STAR_NAMES: dict = {}          # populated once in solver process


def lookup_star(name: str) -> Optional[tuple]:
    return _STAR_NAMES.get(name.strip().lower())


# ---------------------------------------------------------------------------
# Shared state dataclass (passed to all processes via multiprocessing primitives)
# ---------------------------------------------------------------------------

class SharedState:
    """Thin wrapper around multiprocessing primitives."""

    def __init__(self):
        # Camera → solver
        self.frame_shm   = shared_memory.SharedMemory(
            create=True, size=FRAME_BYTES, name=SHM_NAME,
        )
        self.frame_ready = Event()

        # Solver output → LX200
        self.shared_ra   = Value(c_double, 0.0)
        self.shared_dec  = Value(c_double, 0.0)
        self.offset_flag = Value(c_bool,   False)
        self.test_mode   = Value(c_bool,   False)

        # Solver <→ LX200 command channel
        self.cmd_q    = Queue()
        self.result_q = Queue()

        # Solver <→ Camera command channel
        self.cam_cmd_q    = Queue()
        self.cam_result_q = Queue()

        # Maint server <→ Solver
        self.maint_q     = Queue()
        self.maint_res_q = Queue()

    def cleanup(self):
        try:
            self.frame_shm.close()
            self.frame_shm.unlink()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Calibration store
# ---------------------------------------------------------------------------

class CalibrationStore:
    """Thread-safe store for field-angle / boresight calibration data."""

    def __init__(self):
        self._lock   = threading.Lock()
        self._data: dict = {}
        self._path   = "/home/efinder/Solver/calibration.json"
        self._load()

    def _load(self):
        try:
            with open(self._path) as f:
                self._data = json.load(f)
        except Exception:
            self._data = {}

    def _save(self):
        try:
            with open(self._path, "w") as f:
                json.dump(self._data, f, indent=2)
        except Exception as e:
            log.warning("calibration save failed: %s", e)

    def get(self, key, default=None):
        with self._lock:
            return self._data.get(key, default)

    def update(self, **kwargs):
        with self._lock:
            self._data.update(kwargs)
            self._save()

    def reset(self):
        with self._lock:
            self._data = {}
            self._save()

    def snapshot(self) -> dict:
        with self._lock:
            return dict(self._data)


# ---------------------------------------------------------------------------
# Camera process
# ---------------------------------------------------------------------------

def camera_process(
    ss: SharedState,
    config_path: str,
):
    """Process 1 — picamera2 capture loop."""
    log = logging.getLogger("efinder.camera")
    log.info("Camera process starting (pid %d)", os.getpid())

    cfg = load_config(config_path)
    exposure_s = float(cfg.get("Exposure", "1.0"))
    gain       = float(cfg.get("Gain",     "8.0"))

    if Picamera2 is None:
        log.error("picamera2 not available")
        return

    cam = Picamera2()
    # Raw 10-bit Bayer -> ISP -> grey 8-bit
    config = cam.create_still_configuration(
        main={"size": (FRAME_W, FRAME_H), "format": "YUV420"},
        controls={
            "ExposureTime":      int(exposure_s * 1_000_000),
            "AnalogueGain":      gain,
            "AeEnable":          False,
            "AwbEnable":         False,
            "NoiseReductionMode": 0,
        },
    )
    cam.configure(config)
    cam.start()
    log.info("Camera started: exposure=%.3fs gain=%.1f", exposure_s, gain)

    # Attach to shared memory (already created by main)
    shm = shared_memory.SharedMemory(name=SHM_NAME)
    frame_buf = np.ndarray((FRAME_H, FRAME_W), dtype=np.uint8, buffer=shm.buf)

    try:
        while True:
            # Non-blocking drain of camera commands
            while not ss.cam_cmd_q.empty():
                cmd = ss.cam_cmd_q.get_nowait()
                if cmd.get("op") == "set_exp":
                    exposure_s = float(cmd["exposure_s"])
                    gain       = float(cmd.get("gain", gain))
                    cam.set_controls({
                        "ExposureTime": int(exposure_s * 1_000_000),
                        "AnalogueGain": gain,
                    })
                    log.info("Camera: exposure=%.3fs gain=%.1f", exposure_s, gain)
                    ss.cam_result_q.put({"op": "set_exp", "ok": True})

            if ss.test_mode.value:
                time.sleep(0.1)
                continue

            array = cam.capture_array("main")
            # YUV420 — Y plane is the first FRAME_H rows
            grey = array[:FRAME_H, :FRAME_W].copy()
            np.copyto(frame_buf, grey)
            ss.frame_ready.set()

            # Write live JPEG for web UI (contrast-enhanced)
            try:
                pil = Image.fromarray(grey, mode="L")
                pil = ImageEnhance.Contrast(pil).enhance(2.5)
                pil = pil.convert("RGB")
                pil.save(LIVE_JPG, format="JPEG", quality=70)
            except Exception:
                pass

    except KeyboardInterrupt:
        pass
    finally:
        cam.stop()
        shm.close()
        log.info("Camera process exiting")


# ---------------------------------------------------------------------------
# Solver process
# ---------------------------------------------------------------------------

def solver_process(
    ss: SharedState,
    config_path: str,
    cal_store: CalibrationStore,
):
    """Process 2 — tetra3rs extract + solve."""
    global _STAR_NAMES
    log = logging.getLogger("efinder.solver")
    log.info("Solver process starting (pid %d)", os.getpid())

    _STAR_NAMES = _build_starnames(STARS_CSV)
    log.info("Loaded %d named stars", len(_STAR_NAMES))

    cfg = load_config(config_path)
    exposure_s  = float(cfg.get("Exposure", "1.0"))
    gain        = float(cfg.get("Gain",     "8.0"))
    fov_hint    = float(cfg.get("FOV",      "4.0"))   # degrees
    offset_x    = float(cfg.get("OffsetX",  "0.0"))
    offset_y    = float(cfg.get("OffsetY",  "0.0"))

    if tetra3 is None:
        log.error("tetra3 module not available")
        return

    t3 = tetra3.Tetra3()
    log.info("tetra3rs loaded")

    shm = shared_memory.SharedMemory(name=SHM_NAME)
    frame_buf = np.ndarray((FRAME_H, FRAME_W), dtype=np.uint8, buffer=shm.buf)

    last_ra  = 0.0
    last_dec = 0.0
    has_solution = False

    # Rolling stats for solve latency
    solve_times: list = []

    result_store: dict = {
        "solved":   False,
        "ra_deg":   0.0,
        "dec_deg":  0.0,
        "fov_deg":  fov_hint,
        "roll_deg": 0.0,
        "stars":    0,
        "matches":  0,
        "peak":     0,
        "solve_ms": 0,
        "status":   0,
    }

    def _apply_exposure(s: float, g: float, persist: bool):
        nonlocal exposure_s, gain
        exposure_s = s
        gain       = g
        ss.cam_cmd_q.put({"op": "set_exp", "exposure_s": s, "gain": g})
        if persist:
            save_config({"Exposure": str(s), "Gain": str(g)}, config_path)

    def _handle_maint(msg: dict) -> dict:
        nonlocal offset_x, offset_y, fov_hint
        op = msg.get("op", "")

        if op == "ping":
            return {"ok": True}

        elif op == "version":
            return {"ok": True, "version": "tetra3rs_mp-1.0"}

        elif op == "status":
            boresight = {
                "x": CENTRE_X + offset_x,
                "y": CENTRE_Y + offset_y,
            }
            return {
                "ok": True,
                "solution": dict(result_store),
                "boresight": boresight,
                "fov_deg":   fov_hint,
            }

        elif op == "reset_offset":
            offset_x = 0.0
            offset_y = 0.0
            save_config({"OffsetX": "0.0", "OffsetY": "0.0"}, config_path)
            return {"ok": True}

        elif op == "exposure_get":
            return {"ok": True, "exposure_s": exposure_s, "gain": gain}

        elif op == "exposure_set":
            try:
                s = float(msg["exposure_s"])
                g = float(msg.get("gain", gain))
                persist = bool(msg.get("persist", False))
                _apply_exposure(s, g, persist)
                return {"ok": True}
            except (KeyError, ValueError) as e:
                return {"ok": False, "error": str(e)}

        elif op == "gain_set":
            try:
                g = float(msg["gain"])
                persist = bool(msg.get("persist", False))
                _apply_exposure(exposure_s, g, persist)
                return {"ok": True}
            except (KeyError, ValueError) as e:
                return {"ok": False, "error": str(e)}

        elif op == "calibration_status":
            return {"ok": True, **cal_store.snapshot()}

        elif op == "calibration_reset":
            cal_store.reset()
            return {"ok": True}

        elif op == "polar_status":
            return {"ok": True, "polar": None}   # placeholder

        elif op == "polar_start":
            return {"ok": True}

        elif op == "polar_cancel":
            return {"ok": True}

        elif op == "polar_set_latitude":
            return {"ok": True}

        else:
            return {"ok": False, "error": f"unknown op: {op}"}

    try:
        while True:
            # --- Drain maint queue (non-blocking) ---
            while not ss.maint_q.empty():
                msg = ss.maint_q.get_nowait()
                resp = _handle_maint(msg)
                ss.maint_res_q.put(resp)

            # --- Drain LX200 command queue ---
            while not ss.cmd_q.empty():
                cmd = ss.cmd_q.get_nowait()
                op = cmd.get("op", "")
                if op == "goto":
                    # Accept goto target for seeded solve
                    last_ra  = float(cmd.get("ra_deg",  last_ra))
                    last_dec = float(cmd.get("dec_deg", last_dec))
                elif op == "set_offset_flag":
                    ss.offset_flag.value = bool(cmd.get("value", False))
                elif op == "test_mode":
                    ss.test_mode.value = bool(cmd.get("value", False))

            # --- Wait for a new frame (200 ms timeout) ---
            if not ss.frame_ready.wait(timeout=0.2):
                continue
            ss.frame_ready.clear()

            np_img = np.array(frame_buf)  # snapshot

            t0 = time.monotonic()

            # --- Solve ---
            try:
                hint = None
                if has_solution:
                    hint = {"ra": last_ra, "dec": last_dec, "fov": fov_hint}

                sol = t3.solve_from_image(
                    np_img,
                    fov_estimate=fov_hint,
                    fov_max_error=0.5,
                    target_pixel=None,
                    distortion=0,
                    return_matches=True,
                    solve_hint=hint,
                )
            except Exception as e:
                log.debug("solve exception: %s", e)
                sol = None

            elapsed_ms = (time.monotonic() - t0) * 1000

            if sol and sol.get("RA") is not None:
                ra_deg  = float(sol["RA"])
                dec_deg = float(sol["Dec"])
                fov_deg = float(sol.get("FOV", fov_hint))
                roll    = float(sol.get("Roll", 0.0))
                stars   = int(sol.get("Nb_stars", 0))
                matches = int(sol.get("Nb_matches", 0))

                last_ra  = ra_deg
                last_dec = dec_deg
                has_solution = True

                ss.shared_ra.value  = ra_deg
                ss.shared_dec.value = dec_deg

                result_store.update({
                    "solved":   True,
                    "ra_deg":   ra_deg,
                    "dec_deg":  dec_deg,
                    "fov_deg":  fov_deg,
                    "roll_deg": roll,
                    "stars":    stars,
                    "matches":  matches,
                    "peak":     int(np_img.max()),
                    "solve_ms": round(elapsed_ms),
                    "status":   1,
                })

                solve_times.append(elapsed_ms)
                if len(solve_times) > 20:
                    solve_times.pop(0)

                if len(solve_times) % 10 == 0:
                    avg = sum(solve_times) / len(solve_times)
                    log.info(
                        "Solved RA=%.4f Dec=%.4f FOV=%.3f stars=%d ms=%.0f (avg %.0f)",
                        ra_deg, dec_deg, fov_deg, stars, elapsed_ms, avg,
                    )
            else:
                has_solution = False
                result_store.update({
                    "solved":   False,
                    "stars":    0,
                    "peak":     int(np_img.max()),
                    "solve_ms": round(elapsed_ms),
                    "status":   0,
                })

    except KeyboardInterrupt:
        pass
    finally:
        shm.close()
        log.info("Solver process exiting")


# ---------------------------------------------------------------------------
# LX200 / WiFi server process
# ---------------------------------------------------------------------------

RA_RE  = re.compile(r"^:Sr(\d+):(\d+):(\d+)#$")
DEC_RE = re.compile(r"^:Sd([+-]\d+)\*(\d+):(\d+)#$")


def _parse_ra(h, m, s):
    return (int(h) + int(m) / 60.0 + int(s) / 3600.0) * 15.0


def _parse_dec(d, m, s):
    sign = -1 if d.startswith("-") else 1
    d_abs = abs(int(d))
    return sign * (d_abs + int(m) / 60.0 + int(s) / 3600.0)


def _format_ra(deg):
    h = deg / 15.0 % 24.0
    hh = int(h); mm = int((h - hh) * 60); ss = int(round(((h - hh) * 60 - mm) * 60))
    if ss == 60: ss = 0; mm += 1
    if mm == 60: mm = 0; hh = (hh + 1) % 24
    return f"{hh:02d}:{mm:02d}:{ss:02d}#"


def _format_dec(deg):
    sign = "+" if deg >= 0 else "-"
    a = abs(deg)
    dd = int(a); mm = int((a - dd) * 60); ss = int(round(((a - dd) * 60 - mm) * 60))
    if ss == 60: ss = 0; mm += 1
    if mm == 60: mm = 0; dd += 1
    return f"{sign}{dd:02d}*{mm:02d}:{ss:02d}#"


def lx200_handle_client(conn, addr, ss: SharedState):
    log = logging.getLogger("efinder.lx200")
    buf = b""
    pending_ra: Optional[float]  = None
    pending_dec: Optional[float] = None

    try:
        while True:
            data = conn.recv(256)
            if not data:
                break
            buf += data
            while True:
                end = buf.find(b"#")
                if end < 0:
                    break
                cmd_bytes = buf[:end + 1]
                buf = buf[end + 1:]
                cmd = cmd_bytes.decode("ascii", errors="replace")

                resp = b""

                if cmd == ":GR#":
                    resp = _format_ra(ss.shared_ra.value).encode()
                elif cmd == ":GD#":
                    resp = _format_dec(ss.shared_dec.value).encode()
                elif cmd == ":GVP#":
                    resp = b"eFinder#"
                elif cmd == ":GVN#":
                    resp = b"1.0#"
                elif cmd == ":Q#":
                    resp = b""
                elif m := RA_RE.match(cmd):
                    pending_ra = _parse_ra(*m.groups())
                    resp = b"1"
                elif m := DEC_RE.match(cmd):
                    pending_dec = _parse_dec(*m.groups())
                    resp = b"1"
                elif cmd == ":MS#":
                    if pending_ra is not None and pending_dec is not None:
                        ss.cmd_q.put({"op": "goto",
                                      "ra_deg": pending_ra,
                                      "dec_deg": pending_dec})
                        pending_ra = pending_dec = None
                    resp = b"0"
                else:
                    pass   # unrecognised command — no response

                if resp:
                    conn.sendall(resp)
    except Exception as e:
        log.debug("LX200 client %s: %s", addr, e)
    finally:
        conn.close()


def lx200_process(ss: SharedState):
    """Process 3 — LX200/WiFi TCP server."""
    log = logging.getLogger("efinder.lx200")
    log.info("LX200 server starting on %s:%d (pid %d)",
             LX200_HOST, LX200_PORT, os.getpid())

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((LX200_HOST, LX200_PORT))
    server.listen(4)

    try:
        while True:
            try:
                conn, addr = server.accept()
                t = threading.Thread(
                    target=lx200_handle_client,
                    args=(conn, addr, ss),
                    daemon=True,
                )
                t.start()
            except Exception as e:
                log.warning("LX200 accept error: %s", e)
    except KeyboardInterrupt:
        pass
    finally:
        server.close()
        log.info("LX200 server exiting")


# ---------------------------------------------------------------------------
# Maintenance socket server (thread in main process)
# ---------------------------------------------------------------------------

def maint_server_thread(ss: SharedState):
    """Unix-domain socket server for webUI/CLI maintenance commands."""
    log = logging.getLogger("efinder.maint")
    os.makedirs(os.path.dirname(SOCKET_PATH), exist_ok=True)
    try:
        os.unlink(SOCKET_PATH)
    except FileNotFoundError:
        pass

    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(SOCKET_PATH)
    os.chmod(SOCKET_PATH, 0o660)
    srv.listen(8)
    log.info("Maint socket ready at %s", SOCKET_PATH)

    def handle(conn):
        try:
            raw = b""
            while True:
                chunk = conn.recv(4096)
                if not chunk:
                    break
                raw += chunk
                try:
                    msg = json.loads(raw)
                    break
                except json.JSONDecodeError:
                    continue
            if not raw:
                return
            msg = json.loads(raw)
            ss.maint_q.put(msg)
            resp = ss.maint_res_q.get(timeout=5.0)
            conn.sendall(json.dumps(resp).encode())
        except Exception as e:
            try:
                conn.sendall(json.dumps({"ok": False, "error": str(e)}).encode())
            except Exception: pass
        finally:
            conn.close()

    while True:
        try:
            conn, _ = srv.accept()
            threading.Thread(target=handle, args=(conn,), daemon=True).start()
        except Exception as e:
            log.warning("maint accept: %s", e)


# ---------------------------------------------------------------------------
# Process spec + health monitor
# ---------------------------------------------------------------------------

class ProcSpec:
    def __init__(self, name, target, args):
        self.name   = name
        self.target = target
        self.args   = args
        self.proc: Optional[Process] = None

    def start(self):
        self.proc = Process(
            target=self.target,
            args=self.args,
            name=self.name,
            daemon=True,
        )
        self.proc.start()
        log.info("Started %s (pid %d)", self.name, self.proc.pid)

    def alive(self) -> bool:
        return self.proc is not None and self.proc.is_alive()

    def restart(self):
        if self.proc and self.proc.is_alive():
            self.proc.terminate()
            self.proc.join(timeout=3)
        log.warning("Restarting %s", self.name)
        self.start()


def health_monitor(specs: list, interval: float = 10.0):
    """Runs in the main process; restarts dead worker processes."""
    while True:
        time.sleep(interval)
        for spec in specs:
            if not spec.alive():
                log.error("Process %s died — restarting", spec.name)
                spec.restart()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    log.info("eFinder tetra3rs_mp starting (pid %d)", os.getpid())

    config_path = DEFAULT_CONFIG
    cal_store   = CalibrationStore()
    ss          = SharedState()

    specs = [
        ProcSpec("camera", camera_process,  (ss, config_path)),
        ProcSpec("solver", solver_process,  (ss, config_path, cal_store)),
        ProcSpec("lx200",  lx200_process,   (ss,)),
    ]

    for spec in specs:
        spec.start()

    # Maint socket in a daemon thread of the main process
    maint_t = threading.Thread(
        target=maint_server_thread,
        args=(ss,),
        daemon=True,
        name="maint",
    )
    maint_t.start()

    try:
        health_monitor(specs)
    except KeyboardInterrupt:
        log.info("Shutting down")
    finally:
        ss.cleanup()


if __name__ == '__main__':
    set_start_method('fork')
    main()
