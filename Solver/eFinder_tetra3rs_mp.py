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
#   cam_cmd_q      — Queue  solver -> camera: set_exp, capture_once
#   cam_result_q   — Queue  camera -> solver: captured frames
#
# /dev/shm/efinder_state.json  written by solver after each solve — slow
#   telemetry (stars, peak, eTime, etc.) read by lx200 only for diagnostic
#   commands (:GS, :GK, :Gt) which are not timing-critical.

import os
import sys
import math
import socket
import time
import csv
import json
import ctypes
from datetime import datetime
from pathlib import Path
from multiprocessing import (
    Process, Queue, Event, Value,
    shared_memory, set_start_method
)

import numpy as np

# ---------------------------------------------------------------------------
# Shared constants
# ---------------------------------------------------------------------------
home_path   = str(Path.home())
version     = "6.6-tetra3rs-mp-tb6"
config_path = os.path.join(home_path, "Solver/eFinder.config")
solver_path = os.path.join(home_path, "Solver")

FRAME_H  = 760
FRAME_W  = 960
FRAME_SZ = FRAME_H * FRAME_W

CAM_ARCSEC_PX = 50.8
CAM_FOV_DEG   = 13.5

STATE_FILE = '/dev/shm/efinder_state.json'
LIVE_IMAGE = '/dev/shm/efinder_live.jpg'

# tetra3rs centroid convention: origin at image centre, +X right, +Y down.
# Offset stored in eFinder.config as arcminutes on sky; converted to centred
# pixel space for solver queries.
CENTRE_X = FRAME_W / 2.0
CENTRE_Y = FRAME_H / 2.0

# Triple-buffered camera -> solver frame handoff (items 1 + 3).
# Three SharedMemory slots; camera rotates writes across the two slots
# that the solver isn't currently reading. latest_slot and frame_seq are
# published atomically as the final step of each write.
N_FRAME_SLOTS = 3

# ---------------------------------------------------------------------------
# Config helpers
# ---------------------------------------------------------------------------
def load_param():
    param = {}
    if os.path.exists(config_path):
        with open(config_path) as h:
            for line in h:
                parts = line.strip("\n").split(":")
                if len(parts) == 2:
                    param[parts[0]] = str(parts[1])
    return param

def save_param(param):
    with open(config_path, "w") as h:
        for key, value in param.items():
            h.write("%s:%s\n" % (key, value))

# ---------------------------------------------------------------------------
# Coordinate helpers
# ---------------------------------------------------------------------------
class Coordinates:
    def __init__(self):
        self._update_precession_constants()

    def _update_precession_constants(self):
        now  = datetime.now()
        decY = now.year + int(now.strftime('%j')) / 365.25
        self.t = decY - 2000
        self.T = self.t / 100
        self.m  = 3.07496 + 0.00186 * self.T
        self.n2 = 20.0431 - 0.0085  * self.T
        self.n1 = 1.33621 - 0.00057 * self.T

    def dateSet(self, timeOffset, timeStr, dateStr):
        days = 0
        sg   = float(timeOffset)
        hours, minutes, seconds = timeStr.split(':')
        hours = int(hours) + sg
        if hours >= 24:
            hours = str(int(hours - 24)); days = 1
        elif hours < 0:
            hours = str(int(hours + 24)); days = -1
        else:
            hours = str(int(hours))
        timeStr = hours + ':' + minutes + ':' + seconds
        month, day, year = dateStr.split('/')
        day = str(int(day) + days)
        dateStr = month + '/' + day + '/20' + year
        dt_str = dateStr + ' ' + timeStr
        print('Calculated UTC', dt_str)
        os.system('sudo date -u --set "%s"' % dt_str + '.000Z')
        self._update_precession_constants()

    def precess(self, r, d):
        dR = self.m + self.n1 * math.sin(math.radians(r)) * math.tan(math.radians(d))
        dD = self.n2 * math.cos(math.radians(r))
        return r + dR / 240 * self.t, d + dD / 3600 * self.t

    def hh2dms(self, dd):
        minutes, seconds = divmod(abs(dd) * 3600, 60)
        degrees, minutes = divmod(minutes, 60)
        return '%02d:%02d:%02d' % (degrees, minutes, seconds)

    def dd2aligndms(self, dd):
        sign = '+' if dd >= 0 else '-'
        minutes, seconds = divmod(abs(dd) * 3600, 60)
        degrees, minutes = divmod(minutes, 60)
        return '%s%02d*%02d:%02d' % (sign, degrees, minutes, seconds)

    def dd2dms(self, dd):
        sign = '+' if dd >= 0 else '-'
        minutes, seconds = divmod(abs(dd) * 3600, 60)
        degrees, minutes = divmod(minutes, 60)
        return '%s%02d:%02d:%02d' % (sign, degrees, minutes, seconds)

# ---------------------------------------------------------------------------
# Offset / pixel conversion (tetra3rs centred convention)
# d_x / d_y stored in eFinder.config in arcminutes on sky.
# ---------------------------------------------------------------------------
def dxdy2centred(dx_deg, dy_deg):
    """Convert angular offset (degrees) to centred pixel coords."""
    cx =  dx_deg * 3600 / CAM_ARCSEC_PX      # +X right
    cy = -dy_deg * 3600 / CAM_ARCSEC_PX      # +Y down (dy up = negative cy)
    return cx, cy

def centred2dxdy(cx, cy):
    """Convert centred pixel coords to angular offset (degrees)."""
    dx_deg =  cx * CAM_ARCSEC_PX / 3600
    dy_deg = -cy * CAM_ARCSEC_PX / 3600
    return dx_deg, dy_deg

# ---------------------------------------------------------------------------
# CPU affinity helper (item 5)
# ---------------------------------------------------------------------------
# The Pi Zero 2W has 4x Cortex-A53. We pin each worker to specific cores
# to keep the solver's hot numeric loop from being interrupted by camera
# DMA completion handling or by SkySafari socket traffic, and to give the
# solver's two background threads (state writer, live JPEG renderer) room
# to run without stealing cycles from the main solve loop.
#
# Mapping (see CPU_PINNING dict). Tuneable if field measurements suggest
# a different assignment works better.
#
# sched_setaffinity can fail on restricted kernels, inside containers, or
# when systemd CPUAffinity= is already set; we log and carry on. Losing
# the tuning doesn't break correctness.

CPU_PINNING = {
    'main':   {0},      # supervisor — idle most of the time
    'camera': {0},      # light work, shares with USB/SDIO IRQs
    'lx200':  {1},      # isolated from solver so replies never queue
    'solver': {2, 3},   # heavy: main loop + state thread + live-jpeg thread
}

def _pin_cpu(label):
    """Best-effort CPU affinity pin. Logs result, never raises."""
    cores = CPU_PINNING.get(label)
    if not cores:
        return
    try:
        # Verify requested cores actually exist on this box. On non-Pi
        # hardware or odd kernels we just log and skip rather than pin
        # to a phantom core.
        avail = os.sched_getaffinity(0)
        want  = cores & avail
        if not want:
            print('[%s] cpu pin skipped: requested %s not in available %s' %
                  (label, sorted(cores), sorted(avail)))
            return
        os.sched_setaffinity(0, want)
        # Read back what actually stuck (cgroups can narrow the set).
        got = os.sched_getaffinity(0)
        if got == want:
            print('[%s] cpu pin: cores %s' % (label, sorted(got)))
        else:
            print('[%s] cpu pin partial: asked %s, got %s' %
                  (label, sorted(want), sorted(got)))
    except (OSError, AttributeError) as e:
        # AttributeError handles the unlikely case of a Python build
        # without sched_setaffinity (not Linux, not CPython on Linux, etc.)
        print('[%s] cpu pin not supported: %s' % (label, e))

# ===========================================================================
# PROCESS 1 - Camera
# ===========================================================================
def camera_process(shm_names, frame_ready, cam_cmd_q, cam_result_q,
                   test_mode, latest_slot, frame_seq):
    """
    Owns Picamera2. Rotates writes across three SharedMemory slots so the
    solver can read the most-recently-published slot without racing the
    writer. Publishes (latest_slot, frame_seq) atomically after each write.

    shm_names   : list of 3 SharedMemory names (created by main)
    frame_ready : Event — set after each completed publish
    latest_slot : Value(c_int)  — index (0/1/2) of last fully-written slot
    frame_seq   : Value(c_uint64) — monotonic frame counter

    Commands on cam_cmd_q:
        ('set_exp', exposure, gain)
        ('capture_once', None, None)  -> puts ('frame', array) on cam_result_q
        ('stop', None, None)
    """
    _pin_cpu('camera')
    from picamera2 import Picamera2

    param = load_param()

    # Attach to all three SHM slots; wrap each as a numpy view so np.copyto
    # writes straight into shared memory with no intermediate allocation.
    shms      = [shared_memory.SharedMemory(name=n) for n in shm_names]
    slot_bufs = [np.ndarray((FRAME_H, FRAME_W), dtype=np.uint8, buffer=s.buf)
                 for s in shms]

    picam2 = Picamera2()
    cfg = picam2.create_still_configuration(
        main={"size": (FRAME_W, FRAME_H), "format": "YUV420"},
        sensor={"output_size": (2028, 1520)},
        buffer_count=2,
    )
    picam2.configure(cfg)

    def _apply(exp_s, gain):
        picam2.stop()
        picam2.set_controls({
            "AeEnable":     False,
            "AwbEnable":    False,
            "ExposureTime": int(float(exp_s) * 1_000_000),
            "AnalogueGain": int(float(gain)),
        })
        picam2.start()

    _apply(param.get("Exposure", "0.2"), param.get("Gain", "20"))
    test_path = os.path.join(home_path, "Solver/test.npy")

    def _capture():
        if test_mode.value and os.path.exists(test_path):
            return np.load(test_path)
        arr = np.array(picam2.capture_array())
        return arr[0:FRAME_H, 0:FRAME_W]

    def _pick_write_slot(last_published):
        """
        Return a slot index that the solver is guaranteed not to be reading
        right now. With 3 slots and the solver always reading whichever slot
        was most recently published, we just pick any slot other than the
        published one. We prefer to rotate so we don't write to the same
        slot twice in a row — gives the solver a full extra cycle of
        read-safety headroom if it gets preempted mid-copy.
        """
        # Rotate: (published + 1) mod N. The solver may still be reading
        # `published` but not this next slot.
        return (last_published + 1) % N_FRAME_SLOTS

    last_published = -1   # sentinel: first pick goes to slot 0

    print('[camera] ready (triple-buffered)')

    while True:
        # drain on-demand commands
        try:
            while True:
                cmd, a, b = cam_cmd_q.get_nowait()
                if cmd == 'set_exp':
                    _apply(a, b)
                elif cmd == 'capture_once':
                    cam_result_q.put(('frame', _capture().copy()))
                elif cmd == 'stop':
                    picam2.stop()
                    for s in shms: s.close()
                    return
        except Exception:
            pass

        # continuous capture into the next rotation slot
        write_idx = _pick_write_slot(last_published)
        arr = _capture()
        np.copyto(slot_bufs[write_idx], arr)

        # Publish: update latest_slot and bump frame_seq. The order matters:
        # incrementing the sequence *after* updating the slot index means
        # the solver's "read seq, copy, re-read seq" consistency check will
        # correctly detect a mid-copy write.
        with latest_slot.get_lock():
            latest_slot.value = write_idx
        with frame_seq.get_lock():
            frame_seq.value  += 1

        last_published = write_idx
        frame_ready.set()
        time.sleep(0.05)

# ===========================================================================
# PROCESS 2 - Solver (tetra3rs)
# ===========================================================================
def solver_process(shm_names, frame_ready, cam_cmd_q, cam_result_q,
                   lx200_cmd_q, lx200_result_q,
                   shared_ra, shared_dec, offset_flag, test_mode,
                   latest_slot, frame_seq):
    """
    Owns tetra3rs database.
    Continuous loop: wait for frame -> solve -> update shared_ra/shared_dec.
    Also handles on-demand commands from lx200_cmd_q.

    Reads frames from one of three SHM slots; `latest_slot` tells which
    slot is current, `frame_seq` is the monotonic frame number. We track
    `last_solved_seq` so we never redundantly solve the same frame.
    """
    _pin_cpu('solver')
    import tetra3rs
    import serial as _pyserial
    from threading import Thread as _Thread, Lock as _Lock
    from queue import Queue as _ThreadQueue, Empty as _QueueEmpty, Full as _QueueFull
    from PIL import Image, ImageDraw, ImageFont, ImageEnhance, ImageOps

    param = load_param()
    coordinates = Coordinates()

    # Attach to all three SHM slots — we pick which one to copy from at
    # each iteration based on latest_slot's value.
    shms      = [shared_memory.SharedMemory(name=n) for n in shm_names]
    slot_bufs = [np.ndarray((FRAME_H, FRAME_W), dtype=np.uint8, buffer=s.buf)
                 for s in shms]

    # Track last solved frame sequence so we don't re-solve the same frame
    # if the main loop wakes up more often than the camera publishes.
    last_solved_seq = 0

    # --- load database ---
    DB_PATH = os.path.join(home_path, 'Solver/efinder-tetra-database.bin')
    print('[solver] loading tetra3rs database...')
    db = tetra3rs.SolverDatabase.load_from_file(DB_PATH)
    print('[solver] tetra3rs ready  stars=%d  patterns=%d  max_fov=%.1f°' % (
        db.num_stars, db.num_patterns, db.max_fov_deg))

    # --- prewarm (item 9) ---
    # Force Rust-side scratch buffer allocation before the first real frame.
    # Synthesise a tiny list of centroids; we don't care if the solve itself
    # succeeds, only that the extraction/solve code paths have been exercised.
    try:
        _prewarm_img = np.zeros((FRAME_H, FRAME_W), dtype=np.uint8)
        # A handful of bright pixels so extract_centroids has something to find.
        for (y, x) in [(100, 120), (200, 300), (300, 500), (400, 200),
                       (500, 700), (600, 400), (250, 800), (350, 600)]:
            _prewarm_img[y-1:y+2, x-1:x+2] = 255
        _ext = tetra3rs.extract_centroids(_prewarm_img, sigma_threshold=10.0,
                                          max_centroids=20)
        try:
            db.solve_from_centroids(_ext.centroids,
                                    fov_estimate_deg=CAM_FOV_DEG,
                                    fov_max_error_deg=1.0,
                                    image_shape=(FRAME_H, FRAME_W),
                                    solve_timeout_ms=500)
        except Exception:
            pass   # a failed synthetic solve still allocates scratch
        del _prewarm_img, _ext
        print('[solver] prewarm complete')
    except Exception as e:
        print('[solver] prewarm skipped:', e)

    try:
        fnt = ImageFont.truetype(os.path.join(home_path, "Solver/text.ttf"), 16)
    except Exception:
        fnt = ImageFont.load_default()

    # --- FOV calibration ---
    _fov_samples  = []
    _FOV_MIN, _FOV_MAX = 5, 20
    _fov_measured = float(param.get('fov_measured', '0'))

    def _get_fov():
        if _fov_measured > 0 and len(_fov_samples) >= _FOV_MIN:
            return _fov_measured, 0.3
        return CAM_FOV_DEG, 1.0

    def _update_fov(fov_deg):
        nonlocal _fov_measured
        if not fov_deg or fov_deg <= 0: return
        _fov_samples.append(fov_deg)
        if len(_fov_samples) > _FOV_MAX: _fov_samples.pop(0)
        if len(_fov_samples) < _FOV_MIN: return
        avg = sum(_fov_samples) / len(_fov_samples)
        if abs(avg - _fov_measured) > 0.05:
            _fov_measured = avg
            param['fov_measured'] = '%.4f' % avg
            save_param(param)
            print('[solver] FOV calibrated: %.3f deg' % avg)

    # --- offset in centred pixel space (tetra3rs convention) ---
    # d_x / d_y stored as arcminutes in config (on-sky).
    def _build_offset():
        dx_deg = float(param.get("d_x", "0")) / 60.0
        dy_deg = float(param.get("d_y", "0")) / 60.0
        return dxdy2centred(dx_deg, dy_deg)   # (cx, cy) centred pixels

    offset_cx, offset_cy = _build_offset()

    MAX_CENTROIDS          = 20
    SEEDED_TIMEOUT_MS      = 2000
    BLIND_TIMEOUT_MS       = 5000
    RESEED_TOLERANCE_DEG   = 4.0   # how close a new solve must be to trust the seed

    # --- Centroid peak-value attribute probe ---
    # tetra3rs exposes a per-centroid brightness, but the attribute name has
    # drifted across versions (seen: peak_value, peak, intensity, mass).
    # Probe once at startup and cache the resolved attribute name so the
    # hot path is a single getattr rather than a try/except cascade.
    #
    # Fallback of last resort: None — skip peak reporting entirely rather
    # than crash. Peak is used for logging, :GK, and auto_exp heuristics;
    # none of those are required for a correct solve.
    _PEAK_ATTR = None
    try:
        # Synthetic image with a few bright patches so extract_centroids
        # reliably returns at least one Centroid. Use 3x3 blocks rather
        # than single pixels — some centroid extractors require a local
        # neighborhood above threshold, not just one hot pixel.
        _probe_img = np.zeros((40, 40), dtype=np.uint8)
        for (_y, _x) in [(10, 10), (20, 30), (30, 15)]:
            _probe_img[_y-1:_y+2, _x-1:_x+2] = 255
        _probe_extraction = tetra3rs.extract_centroids(
            _probe_img, sigma_threshold=3.0, max_centroids=5,
        )
        if len(_probe_extraction.centroids) > 0:
            _c = _probe_extraction.centroids[0]
            for _name in ('peak_value', 'peak', 'intensity', 'mass'):
                if hasattr(_c, _name):
                    _PEAK_ATTR = _name
                    break
            print('[solver] centroid peak attribute: %r' % _PEAK_ATTR)
        else:
            print('[solver] centroid peak probe: no centroids from synthetic image')
    except Exception as _e:
        print('[solver] centroid peak probe failed:', _e)

    def _centroid_peak(centroid_list):
        """Return the brightest centroid's p
