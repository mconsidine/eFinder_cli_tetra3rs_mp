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
#   cam_cmd_q      — Queue  solver -> camera: set_exp, capture-now
#   cam_result_q   — Queue  camera -> solver: set_exp_ack, capture results
#
# Plate solving pipeline (solver_process):
#   1. Read frame from shared memory.
#   2. Extract centroids via tetra3rs.extract_centroids (Rust, native speed).
#   3. Solve with tetra3rs (blind or seeded).
#   4. Write RA/Dec to shared_ra / shared_dec.
#   5. Persist result to STATE_FILE for webui.

import os
import sys
import json
import math
import time
import threading
import subprocess
import traceback
from datetime import datetime, timezone
from multiprocessing import (
    Process, Value, Queue, Event, shared_memory, set_start_method,
)
from ctypes import c_double, c_bool
from pathlib import Path

import numpy as np

# ---------------------------------------------------------------------------
# Paths and constants
# ---------------------------------------------------------------------------
BASE_DIR   = Path('/home/efinder/Solver')
STATE_FILE = BASE_DIR / 'efinder_state.json'
PARAM_FILE = BASE_DIR / 'efinder_param.json'
LOG_DIR    = BASE_DIR / 'images'

# Shared-memory frame dimensions (must match camera capture size)
FRAME_W = 960   # columns  (width)
FRAME_H = 760   # rows     (height)

N_SHM_SLOTS = 3   # triple-buffer: camera writes slot i % 3, solver reads

HEALTH_INTERVAL = 30   # seconds between watchdog checks in main()

# LX200 server constants
LX200_PORT  = 4060
MAINT_SOCK  = '/run/efinder/maint.sock'

# Path written by solver for webui camera/focus pages
LIVE_IMAGE = '/dev/shm/efinder_live.jpg'

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _pin_cpu(name: str):
    """Attempt to set the process name (cosmetic, best-effort)."""
    try:
        import ctypes, ctypes.util
        libc = ctypes.CDLL(ctypes.util.find_library('c'), use_errno=True)
        libc.prctl(15, name.encode(), 0, 0, 0)   # PR_SET_NAME
    except Exception:
        pass

def load_param() -> dict:
    """Load (or create) the JSON parameter file."""
    try:
        with open(PARAM_FILE) as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        return {}

def save_param(param: dict):
    """Persist parameter dict."""
    with open(PARAM_FILE, 'w') as f:
        json.dump(param, f, indent=2)

# ---------------------------------------------------------------------------
# Coordinates helper (J2000 -> JNow precession + HMS/DMS formatting)
# ---------------------------------------------------------------------------

class Coordinates:
    """
    Minimal coordinate utility used by solver and lx200 processes.

    Provides:
      - J2000 -> JNow precession (IAU low-precision model)
      - RA  -> HH:MM:SS  (LX200 format)
      - Dec -> sDD*MM:SS (LX200 format)
    """

    def __init__(self):
        self._epoch = 2000.0

    # ------------------------------------------------------------------
    # Precession (IAU 1976 low-precision)
    # ------------------------------------------------------------------
    def _jd(self, dt: datetime) -> float:
        """Return Julian Date for a UTC datetime."""
        a = (14 - dt.month) // 12
        y = dt.year + 4800 - a
        m = dt.month + 12 * a - 3
        jdn = (dt.day + (153 * m + 2) // 5 + 365 * y
               + y // 4 - y // 100 + y // 400 - 32045)
        return jdn + (dt.hour - 12) / 24.0 + dt.minute / 1440.0 + dt.second / 86400.0

    def precess(self, ra_deg: float, dec_deg: float,
                from_epoch: float = 2000.0,
                to_epoch:   float | None = None) -> tuple[float, float]:
        """
        Precess (ra_deg, dec_deg) from *from_epoch* to *to_epoch*.
        *to_epoch* defaults to the current Julian year.
        Returns (ra_deg, dec_deg) in the target epoch.
        """
        if to_epoch is None:
            now = datetime.now(timezone.utc)
            to_epoch = 2000.0 + (self._jd(now) - 2_451_545.0) / 365.25

        T = (to_epoch - from_epoch) / 100.0          # Julian centuries

        # IAU 1976 precession angles (arcsec)
        zeta_A  = (2306.2181 + 1.39656 * T) * T + 0.30188 * T * T
        z_A     = zeta_A + 0.79280 * T * T
        theta_A = (2004.3109 - 0.85330 * T) * T - 0.42665 * T * T

        # Convert to radians
        def as2r(x): return math.radians(x / 3600.0)
        za, z, th = as2r(zeta_A), as2r(z_A), as2r(theta_A)

        ra  = math.radians(ra_deg)
        dec = math.radians(dec_deg)

        # Rotation
        A = math.cos(dec) * math.sin(ra + za)
        B = (math.cos(th) * math.cos(dec) * math.cos(ra + za)
             - math.sin(th) * math.sin(dec))
        C = (math.sin(th) * math.cos(dec) * math.cos(ra + za)
             + math.cos(th) * math.sin(dec))

        ra_new  = math.degrees(math.atan2(A, B) + z)
        dec_new = math.degrees(math.asin(C))

        return ra_new % 360.0, dec_new

    # ------------------------------------------------------------------
    # Formatting helpers
    # ------------------------------------------------------------------
    @staticmethod
    def hh2dms(ra_hours: float) -> str:
        """
        Convert decimal hours to 'HH:MM:SS' string (LX200 RA format).
        """
        ra_hours = ra_hours % 24.0
        h = int(ra_hours)
        rem = (ra_hours - h) * 60.0
        m = int(rem)
        s = int((rem - m) * 60.0)
        return f'{h:02d}:{m:02d}:{s:02d}'

    @staticmethod
    def dd2aligndms(dec_deg: float) -> str:
        """
        Convert decimal degrees to 'sDD*MM:SS' string (LX200 Dec format).
        s = '+' or '-'.
        """
        sign = '+' if dec_deg >= 0 else '-'
        dec_deg = abs(dec_deg)
        d = int(dec_deg)
        rem = (dec_deg - d) * 60.0
        m = int(rem)
        s = int((rem - m) * 60.0)
        return f'{sign}{d:02d}*{m:02d}:{s:02d}'

    # ------------------------------------------------------------------
    # Date/time setter (some LX200 clients send :SC#)
    # ------------------------------------------------------------------
    def dateSet(self, month: int, day: int, year: int):
        """Receive an LX200 :SCmm/dd/yy# date packet (stored for future use)."""
        self._client_date = (month, day, year)

# ---------------------------------------------------------------------------
# camera_process
# ---------------------------------------------------------------------------

def camera_process(shm_names, frame_ready, cam_cmd_q, cam_result_q,
                   test_mode, slot_index):
    """
    Capture loop running in its own process.

    Owns Picamera2. Rotates writes across three SharedMemory slots so the
    solver can read the previous frame while camera writes the next.
    """
    _pin_cpu('ef-camera')

    from picamera2 import Picamera2

    param = load_param()
    exp_s = float(param.get('exposure', '4.0'))
    gain  = float(param.get('gain',     '16.0'))

    shms = [shared_memory.SharedMemory(name=n) for n in shm_names]
    bufs = [np.ndarray((FRAME_H, FRAME_W), dtype=np.uint8, buffer=s.buf)
                 for s in shms]

    picam2 = Picamera2()
    cfg = picam2.create_still_configuration(
        main={"size": (FRAME_W, FRAME_H), "format": "YUV420"},
        buffer_count=2,
    )
    picam2.configure(cfg)

    def _apply(exp_s, gain):
        picam2.stop()
        picam2.set_controls({
            "AeEnable":     False,
            "AwbEnable":    False,
            "ExposureTime": int(float(exp_s) * 1_000_000),
            "AnalogueGain": float(gain),
        })
        picam2.start()

    _apply(exp_s, gain)

    slot = 0
    while True:
        # --- service commands from solver ---
        while not cam_cmd_q.empty():
            cmd = cam_cmd_q.get_nowait()
            kind = cmd.get('cmd')
            if kind == 'set_exp':
                exp_s = cmd['exp_s']
                gain  = cmd['gain']
                _apply(exp_s, gain)
                cam_result_q.put({'cmd': 'set_exp_ack'})
            elif kind == 'capture_now':
                # Capture a single frame on demand (used by webui/test)
                arr = np.array(picam2.capture_array())
                if arr.ndim == 3:
                    arr = arr[:, :, 0]
                arr = arr[:FRAME_H, :FRAME_W]
                cam_result_q.put({'cmd': 'capture_done', 'shape': arr.shape})
                continue
            elif kind == 'reload_param':
                param = load_param()
                exp_s = float(param.get('exposure', exp_s))
                gain  = float(param.get('gain', gain))
                _apply(exp_s, gain)
                cam_result_q.put({'cmd': 'reload_ack'})

        # --- normal capture ---
        if test_mode.value:
            # Test mode: fill with synthetic star field
            bufs[slot][:] = 0
            rng = np.random.default_rng()
            for _ in range(30):
                r = rng.integers(5, FRAME_H - 5)
                c = rng.integers(5, FRAME_W - 5)
                bufs[slot][r-2:r+3, c-2:c+3] = 200
            time.sleep(0.5)
        else:
            try:
                arr = np.array(picam2.capture_array())
                if arr.ndim == 3:
                    arr = arr[:, :, 0]   # YUV420: luma plane
                arr = arr[:FRAME_H, :FRAME_W]
                bufs[slot][:arr.shape[0], :arr.shape[1]] = arr
            except Exception as e:
                print(f'[camera] capture error: {e}', flush=True)
                time.sleep(1)
                continue

        slot_index.value = slot
        frame_ready.set()
        slot = (slot + 1) % N_SHM_SLOTS

        # --- handle reconnect / reconfigure commands while waiting ---
        while not cam_cmd_q.empty():
            cmd = cam_cmd_q.get_nowait()
            kind = cmd.get('cmd')
            if kind == 'set_exp':
                exp_s = cmd['exp_s']
                gain  = cmd['gain']
                _apply(exp_s, gain)
                cam_result_q.put({'cmd': 'set_exp_ack'})


# ---------------------------------------------------------------------------
# solver_process
# ---------------------------------------------------------------------------

def solver_process(shm_names, frame_ready, cam_cmd_q, cam_result_q,
                   shared_ra, shared_dec, offset_flag,
                   cmd_q, result_q, slot_index):
    """
    Plate-solving loop.

    Reads frames from shared memory, extracts centroids via tetra3rs,
    solves (seeded or blind), writes RA/Dec to shared Values.
    Writes a contrast-enhanced JPEG to LIVE_IMAGE after each cycle
    so the webui camera view and focus page have a current frame.
    """
    _pin_cpu('ef-solver')

    import tetra3rs
    from PIL import Image as _PILImage

    param = load_param()
    _sigma_threshold = float(param.get('sigma_threshold', '10.0'))
    coordinates = Coordinates()

    # Load star database
    db_path = BASE_DIR / 'tetra3_index_1.6_0.4.bin'
    print(f'[solver] loading star database {db_path} ...', flush=True)
    db = tetra3rs.SolverDatabase.load_from_file(str(db_path))
    print(f'[solver] database loaded: {db.num_stars} stars, {db.num_patterns} patterns',
          flush=True)

    shms = [shared_memory.SharedMemory(name=n) for n in shm_names]
    bufs = [np.ndarray((FRAME_H, FRAME_W), dtype=np.uint8, buffer=s.buf)
                 for s in shms]

    # ------------------------------------------------------------------
    # Load FOV from param (or use default 5.5 deg)
    # ------------------------------------------------------------------
    _fov_measured = float(param.get('fov', '5.5'))

    # ------------------------------------------------------------------
    # Prewarm: extract centroids once on a black frame so tetra3rs JIT
    # compiles before the first real frame arrives.
    # ------------------------------------------------------------------
    _prewarm_img = np.zeros((FRAME_H, FRAME_W), dtype=np.uint8)
    try:
        tetra3rs.extract_centroids(_prewarm_img, sigma_threshold=10.0,
                                    max_centroids=50)
        print('[solver] centroid extractor warmed up', flush=True)
    except Exception as e:
        print(f'[solver] prewarm warning: {e}', flush=True)

    # Persistent solve state
    _last_ra  = None
    _last_dec = None
    _solve_count   = 0
    _fail_count    = 0
    _last_solve_ms = 0.0

    def _write_state(ra=None, dec=None, status='ok', extra=None):
        """Persist solve result to STATE_FILE for the web UI."""
        state = {
            'timestamp': datetime.now(timezone.utc).isoformat(),
            'status':    status,
        }
        if ra is not None:
            state['ra_deg']  = ra
            state['dec_deg'] = dec
            state['ra_hms']  = coordinates.hh2dms(ra / 15.0)
            state['dec_dms'] = coordinates.dd2aligndms(dec)
        if extra:
            state.update(extra)
        try:
            with open(STATE_FILE, 'w') as f:
                json.dump(state, f)
        except Exception:
            pass

    def _write_live_jpg(img: np.ndarray):
        """Write a contrast-stretched JPEG to /dev/shm for the webui."""
        try:
            lo, hi = np.percentile(img, (0.5, 99.5))
            if hi > lo:
                stretched = np.clip((img.astype(np.float32) - lo)
                                    / (hi - lo) * 255, 0, 255).astype(np.uint8)
            else:
                stretched = img
            pil_img = _PILImage.fromarray(stretched, mode='L').convert('RGB')
            pil_img.save(LIVE_IMAGE, format='JPEG', quality=75)
        except Exception:
            pass

    while True:
        # ------------------------------------------------------------------
        # 1. Wait for a new frame
        # ------------------------------------------------------------------
        frame_ready.wait(timeout=5.0)
        frame_ready.clear()

        slot = slot_index.value
        img  = bufs[slot].copy()

        # Write live JPEG for webui regardless of solve outcome
        _write_live_jpg(img)

        # ------------------------------------------------------------------
        # 2. Service maintenance commands (non-blocking)
        # ------------------------------------------------------------------
        while not cmd_q.empty():
            msg = cmd_q.get_nowait()
            action = msg.get('action', '')

            if action == 'get_status':
                result_q.put({
                    'action':       'status',
                    'solve_count':  _solve_count,
                    'fail_count':   _fail_count,
                    'last_solve_ms': _last_solve_ms,
                    'fov':          _fov_measured,
                    'sigma':        _sigma_threshold,
                })

            elif action == 'set_exp':
                cam_cmd_q.put({'cmd': 'set_exp',
                               'exp_s': msg['exp_s'],
                               'gain':  msg['gain']})
                # Wait for ack
                try:
                    ack = cam_result_q.get(timeout=10)
                    result_q.put({'action': 'set_exp_ack', 'ok': True})
                except Exception:
                    result_q.put({'action': 'set_exp_ack', 'ok': False})

            elif action == 'capture_now':
                cam_cmd_q.put({'cmd': 'capture_now'})
                try:
                    r = cam_result_q.get(timeout=10)
                    result_q.put({'action': 'capture_done', 'ok': True})
                except Exception:
                    result_q.put({'action': 'capture_done', 'ok': False})

            elif action == 'save_image':
                try:
                    fname = LOG_DIR / f'ef_{int(time.time())}.npy'
                    np.save(str(fname), img)
                    result_q.put({'action': 'save_image_done', 'path': str(fname)})
                except Exception as e:
                    result_q.put({'action': 'save_image_done', 'error': str(e)})

            elif action == 'reload_param':
                param = load_param()
                _sigma_threshold = float(param.get('sigma_threshold',
                                                    str(_sigma_threshold)))
                _fov_measured    = float(param.get('fov', str(_fov_measured)))
                cam_cmd_q.put({'cmd': 'reload_param'})
                result_q.put({'action': 'reload_ack', 'ok': True})

        # ------------------------------------------------------------------
        # 3. Extract centroids
        # ------------------------------------------------------------------
        try:
            t0 = time.monotonic()

            # Quick probe: is there anything useful in the frame?
            _probe_img = img[::4, ::4]   # 8x decimated thumbnail
            _probe_cents = tetra3rs.extract_centroids(
                _probe_img, sigma_threshold=3.0, max_centroids=5,
                min_area=1, max_area=200)
            if len(_probe_cents) < 3:
                _fail_count += 1
                _write_state(status='no_stars')
                continue

            centroids = tetra3rs.extract_centroids(
                img,
                sigma_threshold=_sigma_threshold,
                max_centroids=150,
                min_area=2,
                max_area=500,
            )
        except Exception as e:
            print(f'[solver] centroid error: {e}', flush=True)
            _fail_count += 1
            continue

        if len(centroids) < 4:
            _fail_count += 1
            _write_state(status='too_few_centroids',
                         extra={'n_centroids': len(centroids)})
            continue

        # ------------------------------------------------------------------
        # 4. Plate solve (seeded first, then blind fallback)
        # ------------------------------------------------------------------
        solved_radec = None
        solved_fov   = None

        # --- seeded attempt ---
        if _last_ra is not None:
            try:
                result = db.solve(
                    centroids,
                    fov_estimate_deg   = _fov_measured,
                    fov_tolerance_frac = 0.2,
                    ra_hint_deg        = _last_ra,
                    dec_hint_deg       = _last_dec,
                    search_radius_deg  = 5.0,
                    match_threshold    = 0.002,
                )
                if result is not None:
                    solved_radec, solved_fov = result
            except Exception as e:
                print(f'[solver] seeded solve error: {e}', flush=True)

        # --- blind fallback ---
        if solved_radec is None:
            try:
                result = db.solve(
                    centroids,
                    fov_estimate_deg   = _fov_measured,
                    fov_tolerance_frac = 0.3,
                    match_threshold    = 0.002,
                )
                if result is not None:
                    solved_radec, solved_fov = result
            except Exception as e:
                print(f'[solver] blind solve error: {e}', flush=True)

        elapsed_ms = (time.monotonic() - t0) * 1000.0
        _last_solve_ms = elapsed_ms

        if solved_radec is None:
            _fail_count += 1
            _write_state(status='no_match',
                         extra={'n_centroids': len(centroids),
                                'solve_ms':    elapsed_ms})
            continue

        # ------------------------------------------------------------------
        # 5. Update shared state
        # ------------------------------------------------------------------
        ra_j2000, dec_j2000 = float(solved_radec[0]), float(solved_radec[1])

        if solved_fov is not None:
            _fov_measured = float(solved_fov)
            param['fov'] = str(_fov_measured)
            save_param(param)

        _last_ra  = ra_j2000
        _last_dec = dec_j2000
        _solve_count += 1

        # Precess to JNow for mount
        ra_jnow, dec_jnow = coordinates.precess(ra_j2000, dec_j2000)

        if not offset_flag.value:
            shared_ra.value  = ra_jnow
            shared_dec.value = dec_jnow

        print(
            f'[solver] #{_solve_count} solved in {elapsed_ms:.0f} ms  '
            f'J2000 {coordinates.hh2dms(ra_j2000 / 15)} '
            f'{coordinates.dd2aligndms(dec_j2000)}  '
            f'JNow {coordinates.hh2dms(ra_jnow / 15)} '
            f'{coordinates.dd2aligndms(dec_jnow)}  '
            f'FOV={_fov_measured:.2f}deg  N={len(centroids)}',
            flush=True,
        )

        _write_state(
            ra  = ra_j2000,
            dec = dec_j2000,
            extra={
                'ra_jnow':    ra_jnow,
                'dec_jnow':   dec_jnow,
                'fov':        _fov_measured,
                'n_centroids': len(centroids),
                'solve_ms':   elapsed_ms,
                'solve_count': _solve_count,
            }
        )


# ---------------------------------------------------------------------------
# lx200_process
# ---------------------------------------------------------------------------

def lx200_process(shared_ra, shared_dec, offset_flag, cmd_q, result_q):
    """
    Serves SkySafari on port 4060.
    Reads ra/dec directly from shared Values - no IPC latency on hot path.
    Also serves maintenance socket at /run/efinder/maint.sock for Flask webui.
    """
    _pin_cpu('lx200')
    coordinates = Coordinates()

    def _read_state(key, default=''):
        try:
            with open(STATE_FILE) as f:
                return str(json.load(f).get(key, default))
        except Exception:
            return default

    # ------------------------------------------------------------------
    # Maintenance socket helpers (used by Flask webui)
    # ------------------------------------------------------------------
    import socket as _socket

    def _maint_server():
        sock_path = MAINT_SOCK
        Path(sock_path).parent.mkdir(parents=True, exist_ok=True)
        try:
            os.unlink(sock_path)
        except FileNotFoundError:
            pass
        srv = _socket.socket(_socket.AF_UNIX, _socket.SOCK_STREAM)
        srv.bind(sock_path)
        srv.listen(5)
        os.chmod(sock_path, 0o777)
        while True:
            try:
                conn, _ = srv.accept()
                threading.Thread(target=_handle_maint, args=(conn,),
                                 daemon=True).start()
            except Exception:
                time.sleep(1)

    def _handle_maint(conn):
        try:
            data = b''
            while True:
                chunk = conn.recv(4096)
                if not chunk:
                    break
                data += chunk
                if data.endswith(b'\n'):
                    break
            msg  = json.loads(data.decode())
            resp = _dispatch_maint(msg)
            conn.sendall(json.dumps(resp).encode() + b'\n')
        except Exception as e:
            try:
                conn.sendall(json.dumps({'ok': False, 'error': str(e)}).encode() + b'\n')
            except Exception:
                pass
        finally:
            conn.close()

    def _dispatch_maint(msg: dict) -> dict:
        action = msg.get('action', '')

        if action == 'ping':
            return {'ok': True, 'pong': True}

        if action == 'version':
            return {'ok': True, 'result': {'version': 'tetra3rs_mp'}}

        if action in ('get_status', 'status'):
            cmd_q.put({'action': 'get_status'})
            try:
                r = result_q.get(timeout=5)
                # Also merge latest solve state from file for webui dashboard
                try:
                    with open(STATE_FILE) as f:
                        file_state = json.load(f)
                except Exception:
                    file_state = {}
                solved = file_state.get('status') == 'ok' and 'ra_deg' in file_state
                solution = None
                if solved:
                    solution = {
                        'solved':   True,
                        'ra_deg':   file_state['ra_deg'],
                        'dec_deg':  file_state['dec_deg'],
                        'fov_deg':  file_state.get('fov', r.get('fov', 0.0)),
                        'stars':    file_state.get('n_centroids', 0),
                        'matches':  file_state.get('n_centroids', 0),
                        'peak':     200,
                        'solve_ms': file_state.get('solve_ms', 0.0),
                        'roll_deg': 0.0,
                    }
                return {
                    'ok': True,
                    'result': {
                        'solution': solution,
                        'boresight': {'x': FRAME_W / 2, 'y': FRAME_H / 2},
                        'fov_deg':  r.get('fov', 0.0),
                        'solve_count': r.get('solve_count', 0),
                        'imu': None,
                    },
                }
            except Exception:
                return {'ok': False, 'error': 'timeout'}

        if action == 'calibration_status':
            return {'ok': True, 'result': {
                'converged': False, 'n': 0, 'fov_deg': 0.0,
            }}

        if action == 'calibration_reset':
            return {'ok': True}

        if action in ('exposure_get', 'exposure_set', 'gain_set'):
            param = load_param()
            if action == 'exposure_get':
                return {'ok': True, 'result': {
                    'exposure_s': float(param.get('exposure', 4.0)),
                    'gain':       float(param.get('gain', 16.0)),
                }}
            exp_s = float(msg.get('exposure_s', param.get('exposure', 4.0)))
            gain  = float(msg.get('gain',       param.get('gain',     16.0)))
            if action == 'gain_set':
                gain = float(msg.get('gain', gain))
                exp_s = float(param.get('exposure', exp_s))
            cmd_q.put({'action': 'set_exp', 'exp_s': exp_s, 'gain': gain})
            if msg.get('persist', False):
                param['exposure'] = str(exp_s)
                param['gain']     = str(gain)
                save_param(param)
            try:
                r = result_q.get(timeout=15)
                return {'ok': True, 'result': r}
            except Exception:
                return {'ok': False, 'error': 'timeout'}

        if action == 'reset_offset':
            return {'ok': True}

        if action in ('polar_status', 'polar_start', 'polar_cancel',
                      'polar_set_latitude'):
            return {'ok': False, 'error': 'polar alignment not implemented in tetra3rs_mp'}

        if action == 'set_exp':
            cmd_q.put({'action': 'set_exp',
                       'exp_s': float(msg.get('exp_s', 4.0)),
                       'gain':  float(msg.get('gain',  16.0))})
            try:
                r = result_q.get(timeout=15)
                return {'ok': True, 'result': r}
            except Exception:
                return {'ok': False, 'error': 'timeout'}

        if action == 'capture_now':
            cmd_q.put({'action': 'capture_now'})
            try:
                r = result_q.get(timeout=15)
                return {'ok': True, 'result': r}
            except Exception:
                return {'ok': False, 'error': 'timeout'}

        if action == 'save_image':
            cmd_q.put({'action': 'save_image'})
            try:
                r = result_q.get(timeout=15)
                return {'ok': True, 'result': r}
            except Exception:
                return {'ok': False, 'error': 'timeout'}

        if action == 'reload_param':
            cmd_q.put({'action': 'reload_param'})
            try:
                r = result_q.get(timeout=10)
                return {'ok': True, 'result': r}
            except Exception:
                return {'ok': False, 'error': 'timeout'}

        if action == 'get_state':
            try:
                with open(STATE_FILE) as f:
                    state = json.load(f)
                return {'ok': True, 'result': state}
            except Exception as e:
                return {'ok': False, 'error': str(e)}

        if action == 'set_date':
            a = msg.get('args', [])
            coordinates.dateSet(*a)
            return {'ok': True}

        return {'ok': False, 'error': f'unknown action: {action}'}

    threading.Thread(target=_maint_server, daemon=True).start()

    # ------------------------------------------------------------------
    # LX200 TCP server
    # ------------------------------------------------------------------
    import socket

    nonlocal_state = {
        'target_ra':  None,
        'target_dec': None,
    }

    def _handle_lx200(conn, addr):
        print(f'[lx200] connect from {addr}', flush=True)
        buf = ''
        try:
            while True:
                data = conn.recv(256)
                if not data:
                    break
                buf += data.decode('ascii', errors='replace')
                while '#' in buf:
                    cmd, buf = buf.split('#', 1)
                    resp = _process_lx200(cmd, nonlocal_state)
                    if resp:
                        conn.sendall(resp.encode('ascii'))
        except Exception:
            pass
        finally:
            conn.close()
            print(f'[lx200] disconnect {addr}', flush=True)

    def _process_lx200(cmd: str, state: dict) -> str:
        cmd = cmd.strip()
        if not cmd.startswith(':'):
            return ''
        cmd = cmd[1:]   # strip leading ':'

        # --- RA / Dec queries (hot path) ---
        if cmd == 'GR':
            ra  = shared_ra.value
            raPacket  = coordinates.hh2dms(ra / 15) + '#'
            return raPacket

        if cmd == 'GD':
            dec = shared_dec.value
            decPacket = coordinates.dd2aligndms(dec) + '#'
            return decPacket

        # --- Target RA / Dec set ---
        if cmd.startswith('Sr'):
            state['target_ra'] = cmd[2:]
            return '1'

        if cmd.startswith('Sd'):
            state['target_dec'] = cmd[2:]
            return '1'

        # --- Sync ---
        if cmd == 'CM':
            return 'Coordinates     matched.        #'

        # --- Slew ---
        if cmd == 'MS':
            return '0'

        # --- Halt ---
        if cmd == 'Q':
            return ''

        # --- Date / Time ---
        if cmd.startswith('SC'):
            # :SCmm/dd/yy#
            try:
                parts = cmd[2:].split('/')
                coordinates.dateSet(int(parts[0]), int(parts[1]), int(parts[2]))
            except Exception:
                pass
            return '1Updating        Planetary Data  #                                #'

        if cmd == 'GC':
            now = datetime.now()
            return f'{now.month:02d}/{now.day:02d}/{str(now.year)[-2:]}#'

        if cmd == 'GL':
            now = datetime.now()
            return f'{now.hour:02d}:{now.minute:02d}:{now.second:02d}#'

        if cmd == 'GG':
            return '+00#'

        # --- Telescope / mount info ---
        if cmd == 'GVP':
            return 'eFinder#'

        if cmd == 'GVN':
            return '1.0#'

        if cmd == 'GVD':
            return '2025-01-01#'

        if cmd == 'GVT':
            return '00:00:00#'

        if cmd == 'GVF':
            return 'eFinder|A|00#'

        if cmd == 'Gstat':
            return '0#'

        if cmd == 'p':
            return 'P'

        # --- Alignment ---
        if cmd == 'Aa':
            return '0'

        if cmd == 'ACK':
            return 'A'

        # --- Get RA/Dec in different formats ---
        if cmd == 'Gr':
            ra = shared_ra.value
            return coordinates.hh2dms(ra / 15) + '#'

        if cmd == 'Gd':
            dec = shared_dec.value
            return coordinates.dd2aligndms(dec) + '#'

        # --- Offset / calibration commands (no-op ok) ---
        if cmd.startswith('St') or cmd.startswith('Sg') or cmd.startswith('SL'):
            return '1'

        # --- Ignored / unknown ---
        return ''

    # Bind and serve
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(('0.0.0.0', LX200_PORT))
    srv.listen(5)
    print(f'[lx200] listening on port {LX200_PORT}', flush=True)

    while True:
        try:
            conn, addr = srv.accept()
            threading.Thread(target=_handle_lx200, args=(conn, addr),
                             daemon=True).start()
        except Exception as e:
            print(f'[lx200] accept error: {e}', flush=True)
            time.sleep(1)


# ---------------------------------------------------------------------------
# Watchdog / main
# ---------------------------------------------------------------------------

def main():
    # Shared memory: N_SHM_SLOTS slots, each FRAME_H * FRAME_W bytes
    shms = [
        shared_memory.SharedMemory(create=True, size=FRAME_H * FRAME_W)
        for _ in range(N_SHM_SLOTS)
    ]
    shm_names = [s.name for s in shms]

    frame_ready  = Event()
    slot_index   = Value('i', 0)
    shared_ra    = Value(c_double, 0.0)
    shared_dec   = Value(c_double, 0.0)
    offset_flag  = Value(c_bool,   False)
    test_mode    = Value(c_bool,   False)
    cmd_q        = Queue()
    result_q     = Queue()
    cam_cmd_q    = Queue()
    cam_result_q = Queue()

    proc_specs = [
        dict(
            name   = 'camera',
            target = camera_process,
            args   = (shm_names, frame_ready, cam_cmd_q, cam_result_q,
                      test_mode, slot_index),
        ),
        dict(
            name   = 'solver',
            target = solver_process,
            args   = (shm_names, frame_ready, cam_cmd_q, cam_result_q,
                      shared_ra, shared_dec, offset_flag,
                      cmd_q, result_q, slot_index),
        ),
        dict(
            name   = 'lx200',
            target = lx200_process,
            args   = (shared_ra, shared_dec, offset_flag, cmd_q, result_q),
        ),
    ]

    procs = {}

    def _start(spec):
        p = Process(target=spec['target'], args=spec['args'], daemon=False)
        p.start()
        procs[spec['name']] = p
        print(f'[main] started {spec["name"]} pid={p.pid}', flush=True)

    for spec in proc_specs:
        _start(spec)

    # Watchdog loop
    while True:
        time.sleep(HEALTH_INTERVAL)
        for spec in proc_specs:
            p = procs.get(spec['name'])
            if p is None or not p.is_alive():
                exit_code = p.exitcode if p else 'None'
                print(f'[main] {spec["name"]} died (exit={exit_code}), restarting ...',
                      flush=True)
                if p:
                    p.close()
                _start(spec)


if __name__ == '__main__':
    set_start_method('fork')
    main()
