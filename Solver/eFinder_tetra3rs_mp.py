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
#   cam_cmd_q      — Queue  solver -> camera: set_exp, capture_now
#   shared_cfg     — Manager().dict() IMU state + calibration, all workers
#
# Coordinate system: J2000 RA/Dec throughout. LX200 protocol conversion in lx200
# worker only.
#
# Usage: python3 eFinder_tetra3rs_mp.py [--help]

import argparse
import array
import ast
import csv
import datetime
import json
import logging
import math
import multiprocessing as mp
import os
import re
import shutil
import socket
import struct
import subprocess
import sys
import threading
import time
import traceback
from ctypes import c_bool, c_double
from multiprocessing import shared_memory
from pathlib import Path

import numpy as np

try:
    import tetra3rs
except ImportError:
    print('[FATAL] tetra3rs not found — install the Rust/PyO3 extension', flush=True)
    sys.exit(1)

try:
    from picamera2 import Picamera2
except ImportError:
    Picamera2 = None  # allow import for unit-testing on non-Pi hosts

try:
    from imu_math import quat_delta_rotvec as _quat_delta_rotvec
    _IMU_MATH_OK = True
except ImportError:
    _IMU_MATH_OK = False
    def _quat_delta_rotvec(q_now, q_ref):
        return (0.0, 0.0, 0.0)

# ── logging ────────────────────────────────────────────────────────────────────────────────────
logging.basicConfig(
    level   = logging.INFO,
    format  = '%(asctime)s %(processName)-14s %(levelname)s %(message)s',
    datefmt = '%H:%M:%S',
)
log = logging.getLogger(__name__)

# ── paths ──────────────────────────────────────────────────────────────────────────────────
home_path  = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
_param_path = os.path.join(home_path, 'Solver/eFinder.config')

# ── constants ────────────────────────────────────────────────────────────────────────────────────────────────────
FRAME_H         = 760
FRAME_W         = 960
FRAME_BYTES     = FRAME_H * FRAME_W        # uint8 greyscale
MAX_CENTROIDS   = 50
CAPTURE_TIMEOUT = 8.0   # seconds
HEALTH_POLL    = 10.0   # seconds between health checks
RESTART_LIMIT   = 3     # max automatic restarts per worker

# ── shared parameter helpers ──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
def load_param():
    """Load key=value config; return dict."""
    p = {}
    try:
        with open(_param_path) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'): continue
                if '=' in line:
                    k, _, v = line.partition('=')
                    p[k.strip()] = v.strip()
    except FileNotFoundError:
        pass
    return p

def save_param(p):
    os.makedirs(os.path.dirname(_param_path), exist_ok=True)
    with open(_param_path, 'w') as f:
        for k, v in sorted(p.items()):
            f.write('%s=%s\n' % (k, v))

# ── catalogue helpers ───────────────────────────────────────────────────────────────────────────────────────────────────────────────
def load_catalogue():
    """Return list-of-dicts from Solver/catalogue.csv (ra,dec,hipId columns)."""
    cat = []
    path = os.path.join(home_path, 'Solver/catalogue.csv')
    try:
        with open(path) as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    cat.append({
                        'ra':  float(row['ra']),
                        'dec': float(row['dec']),
                        'id':  str(row.get('hipId', row.get('id', ''))),
                    })
                except (ValueError, KeyError):
                    pass
    except Exception:
        pass
    return cat


def _imu_predict(shared_cfg):
    """
    Return IMU dead-reckoning (ra_deg, dec_deg) or None.

    Uses the 2x3 calibration matrix built passively from consecutive
    plate-solves.  Falls through to None whenever the IMU is absent,
    data is stale, calibration is insufficient, or the predicted delta
    exceeds the 5-degree safety cap.
    """
    if not shared_cfg.get('imu_available', False):
        return None
    if shared_cfg.get('imu_calib_n', 0) < 3:
        return None
    if shared_cfg.get('imu_calib_quality', 0.0) < 0.85:
        return None

    q_now = shared_cfg.get('imu_q')
    imu_t = shared_cfg.get('imu_t', 0.0)
    if q_now is None or time.monotonic() - imu_t > 2.0:
        return None

    q_ref   = shared_cfg.get('imu_ref_q')
    ra_ref  = shared_cfg.get('imu_ref_ra_deg')
    dec_ref = shared_cfg.get('imu_ref_dec_deg')
    ref_t   = shared_cfg.get('imu_ref_t', 0.0)
    if q_ref is None or ra_ref is None or time.monotonic() - ref_t > 120.0:
        return None

    C_flat = shared_cfg.get('imu_calib_C')
    if C_flat is None or len(C_flat) != 6:
        return None

    try:
        r = _quat_delta_rotvec(q_now, q_ref)
    except Exception:
        return None

    c  = C_flat
    dr = c[0]*r[0] + c[1]*r[1] + c[2]*r[2]   # camera-right
    du = c[3]*r[0] + c[4]*r[1] + c[5]*r[2]   # camera-up

    roll_ref = shared_cfg.get('imu_ref_roll_deg', 0.0)
    roll_rad = math.radians(roll_ref)
    cos_r, sin_r = math.cos(roll_rad), math.sin(roll_rad)
    dra_rad  =  dr * cos_r - du * sin_r
    ddec_rad =  dr * sin_r + du * cos_r

    if abs(dra_rad) > math.radians(5.0) or abs(ddec_rad) > math.radians(5.0):
        return None

    cos_dec = math.cos(math.radians(dec_ref))
    if abs(cos_dec) < 0.01:
        return None  # within ~0.6 deg of a pole

    ra_pred  = (ra_ref  + math.degrees(dra_rad)  / cos_dec) % 360.0
    dec_pred = max(-90.0, min(90.0, dec_ref + math.degrees(ddec_rad)))
    return ra_pred, dec_pred


# ── OTA update ──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
def check_update():
    """Return (new_version_str, tarball_url) or (None, None)."""
    manifest = os.path.join(home_path, 'update_manifest.json')
    try:
        with open(manifest) as f:
            m = json.load(f)
        current = m.get('current_version', '0.0.0')
        latest  = m.get('latest_version',  '0.0.0')
        url     = m.get('download_url', '')
        if latest != current and url:
            return latest, url
    except Exception:
        pass
    return None, None

def apply_update(url):
    tmp = '/tmp/efinder_update.tar.gz'
    subprocess.run(['wget', '-q', '-O', tmp, url], check=True, timeout=60)
    subprocess.run(['tar', '-xzf', tmp, '-C', home_path], check=True)
    log.info('[main] OTA update applied from %s', url)

# ── proc_specs restart registry ───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
restart_counts = {}

def _start_worker(spec, shared):
    """
    Start a worker process from a proc_spec dict.
    spec keys: name, target, args (optional extra positional args)
    shared: tuple of shared objects prepended to args.
    Returns multiprocessing.Process.
    """
    name = spec['name']
    restart_counts.setdefault(name, 0)
    extra = spec.get('args', ())
    p = mp.Process(
        target = spec['target'],
        args   = shared + tuple(extra),
        name   = name,
        daemon = True,
    )
    p.start()
    log.info('[main] started %s pid=%d', name, p.pid)
    return p

# ───────────────────────────────────────────────────────────────────────────────────
# PROCESS 1 — camera
# ───────────────────────────────────────────────────────────────────────────────────

def camera_worker(
    frame_shm_name, frame_ready, keep,
    test_mode, cam_cmd_q,
    *_extra,
):
    """
    Capture loop.  Writes greyscale frames to shared memory, then signals
    frame_ready.  Obeys cam_cmd_q for set_exp / capture_now.
    """
    import signal
    signal.signal(signal.SIGINT, signal.SIG_IGN)

    shm  = shared_memory.SharedMemory(name=frame_shm_name)
    buf  = np.ndarray((FRAME_H, FRAME_W), dtype=np.uint8, buffer=shm.buf)

    param = load_param()
    exposure = float(param.get('Exposure', '0.2'))
    gain     = float(param.get('Gain', '20'))

    cam = None
    if Picamera2 is not None and not test_mode.value:
        try:
            cam = Picamera2()
            cfg = cam.create_still_configuration(
                main={'size': (FRAME_W, FRAME_H), 'format': 'YUV420'},
                buffer_count=2,
            )
            cam.configure(cfg)
            cam.set_controls({'ExposureTime': int(exposure * 1e6), 'AnalogueGain': gain})
            cam.start()
        except Exception as e:
            log.warning('[camera] picamera2 init failed: %s', e)
            cam = None

    log.info('[camera] started, test_mode=%s', test_mode.value)

    def _apply_set_exp(exp, gn, persist=True):
        nonlocal exposure, gain
        exposure, gain = float(exp), float(gn)
        if cam:
            cam.set_controls({'ExposureTime': int(exposure * 1e6), 'AnalogueGain': gain})

    capture_event = threading.Event()
    capture_result = [None]

    def _capture_frame():
        if cam:
            arr = cam.capture_array('main')
            # YUV420 -> Y plane
            buf[:] = arr[:FRAME_H, :FRAME_W]
        else:
            # synthetic star field for testing
            buf[:] = np.random.randint(0, 30, (FRAME_H, FRAME_W), dtype=np.uint8)
            rng = np.random.default_rng()
            for _ in range(30):
                r = rng.integers(5, FRAME_H - 5)
                c = rng.integers(5, FRAME_W - 5)
                buf[r-2:r+3, c-2:c+3] = rng.integers(180, 255)

    while keep.value:
        # drain cmd queue
        while True:
            try:
                cmd = cam_cmd_q.get_nowait()
            except Exception:
                break
            if cmd[0] == 'set_exp':
                _apply_set_exp(cmd[1], cmd[2])
            elif cmd[0] == 'capture_now':
                capture_event.set()

        _capture_frame()
        frame_ready.set()
        time.sleep(max(0.05, exposure))

    if cam:
        cam.stop()
        cam.close()
    shm.close()
    log.info('[camera] exiting')


# ───────────────────────────────────────────────────────────────────────────────────
# PROCESS 2 — solver
# ───────────────────────────────────────────────────────────────────────────────────

def solver_worker(
    frame_shm_name, frame_ready,
    shared_ra, shared_dec, offset_flag,
    keep, test_mode,
    cmd_q, result_q, cam_cmd_q,
    shared_cfg,
    *_extra,
):
    import signal
    signal.signal(signal.SIGINT, signal.SIG_IGN)

    from calibration import FovCalibrator
    from polar_run import PolarAligner, PolarState

    # Start IMU daemon thread (no-op if smbus2/BNO055 absent)
    try:
        from imu_proc import start_imu_thread
        start_imu_thread(shared_cfg)
    except Exception as _e:
        log.warning('[solver] IMU thread not started: %s', _e)

    shm  = shared_memory.SharedMemory(name=frame_shm_name)
    frame = np.ndarray((FRAME_H, FRAME_W), dtype=np.uint8, buffer=shm.buf)

    param = load_param()

    # ── calibrator (FovCalibrator requires param, save_param_fn, fov_estimate, frame_width) ──
    _calibrator = FovCalibrator(
        param,
        save_param,
        float(param.get('FOV', '5.0')),
        FRAME_W,
    )

    # ── polar aligner ──
    _polar_lat = None
    try:
        _polar_lat = float(param.get('Latitude', ''))
    except (ValueError, TypeError):
        pass
    _polar = PolarAligner(latitude_deg=_polar_lat)

    # ── solver state ──
    solved_radec   = None    # (ra_deg, dec_deg) J2000
    solve          = True    # actively solving when True
    offset_cx      = offset_cy = 0.0
    offset_str     = 'no offset'
    frame_n        = 0
    _fov_measured  = None   # degrees
    _detect_sigma  = float(param.get('Sigma', '8'))

    # ── result JSON path ──
    result_path = os.path.join(home_path, 'Solver/solve_result.json')

    # ── tetra3rs solver handle ──
    cat_path = os.path.join(home_path, 'Solver/tetra3_index_1.6_0.4.bin')
    try:
        solver = tetra3rs.Tetra3(cat_path)
    except Exception as e:
        log.error('[solver] failed to load catalogue: %s', e)
        shm.close(); return

    # ── catalogue for seeded solve ──
    catalogue = load_catalogue()

    # ── IMU calibration state (accumulates solve-pair training data) ──
    _imu_calib_state = {
        'pairs':     [],   # list of (rotvec_3, cam_r, cam_u)
        'prev_q':    None,
        'prev_ra':   None,
        'prev_dec':  None,
        'prev_roll': None,
    }

    def _refit_imu_calib():
        """Least-squares fit of the 2x3 IMU->camera calibration matrix."""
        pairs = _imu_calib_state['pairs']
        n = len(pairs)
        shared_cfg['imu_calib_n'] = n
        if n < 3:
            return
        try:
            X = np.array([[rv[0], rv[1], rv[2]] for rv, cr, cu in pairs])
            Y = np.array([[cr, cu]               for rv, cr, cu in pairs])
            C_T = np.linalg.lstsq(X, Y, rcond=None)[0]  # (3, 2)
            C   = C_T.T                                   # (2, 3)
            Y_pred  = X @ C_T
            ss_res  = float(np.sum((Y - Y_pred) ** 2))
            ss_tot  = float(np.sum((Y - Y.mean(axis=0)) ** 2))
            quality = float(1.0 - ss_res / ss_tot) if ss_tot > 1e-12 else 0.0
            shared_cfg['imu_calib_C']       = C.ravel().tolist()
            shared_cfg['imu_calib_quality'] = max(0.0, min(1.0, quality))
            log.debug('[solver] imu calib n=%d quality=%.3f', n, quality)
        except Exception as _e:
            log.debug('[solver] imu refit error: %s', _e)

    # ── helpers ──

    def _request_capture():
        """Signal camera and wait for fresh frame; return copy."""
        frame_ready.clear()
        cam_cmd_q.put(('capture_now',))
        if not frame_ready.wait(CAPTURE_TIMEOUT):
            raise TimeoutError('frame capture timed out')
        return frame.copy()

    def _centroid_peak(centroids, img):
        if not centroids:
            return 0
        peaks = []
        for cx, cy, *_ in centroids:
            r, c = int(cy), int(cx)
            patch = img[max(0,r-2):r+3, max(0,c-2):c+3]
            peaks.append(int(patch.max()) if patch.size else 0)
        return max(peaks)

    def _do_solve(img):
        nonlocal solved_radec, _fov_measured
        ext = tetra3rs.extract_centroids(
            img,
            sigma_threshold = _detect_sigma,
            max_centroids   = MAX_CENTROIDS,
        )

        fov_hint  = _calibrator.get_fov_estimate()
        ra_hint   = dec_hint = None
        if solved_radec:
            ra_hint, dec_hint = solved_radec

        # seeded solve first if we have a prior position
        result = None
        if ra_hint is not None and fov_hint is not None:
            try:
                result = solver.solve_from_centroids(
                    ext,
                    (FRAME_H, FRAME_W),
                    fov_estimate   = fov_hint,
                    fov_max_error  = 0.2,
                    target_ra      = ra_hint,
                    target_dec     = dec_hint,
                    search_radius  = 5.0,
                    match_threshold= 1e-5,
                )
            except Exception:
                result = None

        if result is None:
            # blind solve
            try:
                result = solver.solve_from_centroids(
                    ext,
                    (FRAME_H, FRAME_W),
                    fov_estimate  = _calibrator.get_fov_estimate() or float(param.get('FOV', '5.0')),
                    fov_max_error = _calibrator.get_fov_max_error(),
                    match_threshold=1e-6,
                )
            except Exception as e:
                log.debug('[solver] blind solve error: %s', e)
                return False

        if result is None:
            return False

        ra_deg  = result.ra
        dec_deg = result.dec
        roll_deg = getattr(result, 'roll', 0.0) or 0.0
        if hasattr(result, 'fov'):
            _fov_measured = result.fov
            _calibrator.update_from_solve(result.fov)

        # apply offset
        if offset_flag.value and (offset_cx or offset_cy):
            plate_scale = (_fov_measured or float(param.get('FOV','5.0'))) / FRAME_H  # deg/px
            dec_deg += offset_cy * plate_scale
            ra_deg  += offset_cx * plate_scale / max(0.01, math.cos(math.radians(dec_deg)))

        solved_radec        = (ra_deg, dec_deg)
        shared_ra.value     = ra_deg
        shared_dec.value    = dec_deg

        _stars  = len(ext.centroids) if hasattr(ext, 'centroids') else 0
        _peak   = _centroid_peak(ext.centroids, img) if hasattr(ext, 'centroids') else 0
        _matches = getattr(result, 'matches', 0) or getattr(result, 'num_matches', 0)

        # ── identify nearest catalogue star ──
        name = sn = hipId = ''
        if catalogue:
            ra_r  = math.radians(ra_deg)
            dec_r = math.radians(dec_deg)
            best  = None
            for star in catalogue:
                d = math.acos(max(-1.0, min(1.0,
                    math.sin(dec_r)*math.sin(math.radians(star['dec'])) +
                    math.cos(dec_r)*math.cos(math.radians(star['dec'])) *
                    math.cos(ra_r - math.radians(star['ra']))
                )))
                if best is None or d < best[0]:
                    best = (d, star)
            if best and best[0] < math.radians(0.5):
                cat_id = best[1]['id']
                name, sn, hipId = _lookup_star(cat_id)

        # ── write result JSON ──
        ra_h  = ra_deg  / 15.0
        ra_hms  = '%02dh%02dm%05.2fs' % (int(ra_h), int((ra_h%1)*60), ((ra_h*60)%1)*60)
        dec_dms = '%+03dd%02dm%05.2fs' % (int(dec_deg), int(abs(dec_deg)%1*60),
                                           (abs(dec_deg)*60%1)*60)
        payload = {
            'ra_deg':   ra_deg,
            'dec_deg':  dec_deg,
            'ra':       ra_hms,
            'dec':      dec_dms,
            'star':     '%s%s' % (name, sn) if name else '',
            'hip':      hipId,
            'frame':    frame_n,
            'ts':       datetime.datetime.utcnow().isoformat(),
            'offset':   offset_str,
            'fov':      round(_fov_measured, 4) if _fov_measured else None,
            'solved':   True,
            'stars':    _stars,
            'peak':     _peak,
            'matches':  _matches,
            'solve_ms': 0,
            'roll_deg': roll_deg,
            'status':   1,
        }
        try:
            with open(result_path, 'w') as f:
                json.dump(payload, f)
        except Exception:
            pass

        # ── update IMU dead-reckoning reference and accumulate training pair ──
        if shared_cfg.get('imu_available', False):
            q_now = shared_cfg.get('imu_q')
            if q_now is not None:
                t_now = time.monotonic()
                # Always update the reference quaternion (origin for dead-reckoning)
                shared_cfg['imu_ref_q']        = q_now
                shared_cfg['imu_ref_ra_deg']   = ra_deg
                shared_cfg['imu_ref_dec_deg']  = dec_deg
                shared_cfg['imu_ref_t']        = t_now
                shared_cfg['imu_ref_roll_deg'] = roll_deg
                # Accumulate a calibration training pair from the previous solve
                if _imu_calib_state['prev_q'] is not None:
                    try:
                        rv = _quat_delta_rotvec(q_now, _imu_calib_state['prev_q'])
                        dra_deg  = ((ra_deg  - _imu_calib_state['prev_ra']  + 180) % 360) - 180
                        ddec_deg =   dec_deg - _imu_calib_state['prev_dec']
                        # Only include pairs with sane deltas (< 10 deg)
                        if abs(dra_deg) < 10 and abs(ddec_deg) < 10:
                            pr       = math.radians(_imu_calib_state['prev_roll'] or 0.0)
                            cos_dec  = math.cos(math.radians(dec_deg))
                            dra_rad  = math.radians(dra_deg) * cos_dec  # arc-length
                            ddec_rad = math.radians(ddec_deg)
                            cam_r =  dra_rad * math.cos(pr) + ddec_rad * math.sin(pr)
                            cam_u = -dra_rad * math.sin(pr) + ddec_rad * math.cos(pr)
                            _imu_calib_state['pairs'].append((rv, cam_r, cam_u))
                            if len(_imu_calib_state['pairs']) > 50:
                                _imu_calib_state['pairs'].pop(0)
                            _refit_imu_calib()
                    except Exception as _e:
                        log.debug('[solver] imu calib pair error: %s', _e)
                _imu_calib_state['prev_q']    = q_now
                _imu_calib_state['prev_ra']   = ra_deg
                _imu_calib_state['prev_dec']  = dec_deg
                _imu_calib_state['prev_roll'] = roll_deg

        return True

    def _set_camera(exp, gain, persist=True):
        param['Exposure'] = str(exp)
        param['Gain']     = str(gain)
        if persist:
            save_param(param)
        cam_cmd_q.put(('set_exp', exp, gain))

    _star_name_dict = {}
    try:
        with open(os.path.join(home_path, 'Solver/starnames.csv')) as _f:
            for _row in csv.reader(_f):
                if len(_row) >= 3:
                    _star_name_dict[str(_row[1]).strip()] = (
                        _row[0].strip(),
                        (' (%s)' % _row[2].strip()) if _row[2].strip() else '',
                    )
    except Exception:
        pass

    def _lookup_star(catalog_id):
        hipId = str(abs(int(catalog_id)))
        name, sn = _star_name_dict.get(hipId, ('', ''))
        return name, sn, hipId

    def _handle(cmd, a, b):
        nonlocal solve, solved_radec, offset_cx, offset_cy, offset_str
        nonlocal keep, frame_n, _fov_measured, _detect_sigma

        if cmd == 'adj_exp':
            new_exp = '%.1f' % max(0.1, float(param.get('Exposure','0.2'))
                                   + float(a) * 0.1)
            _set_camera(new_exp, param.get('Gain','20'))
            return new_exp

        elif cmd == 'set_exp':
            persist = bool(b) if b is not None else True
            _set_camera(a, param.get('Gain', '20'), persist=persist)
            return 'ok'

        elif cmd == 'get_param':
            return json.dumps(param)

        elif cmd == 'set_param':
            k, v = str(a), str(b)
            param[k] = v
            save_param(param)
            return 'ok'

        elif cmd == 'get_result':
            try:
                with open(result_path) as f:
                    return f.read()
            except Exception:
                return '{}'

        elif cmd == 'solve_now':
            try:
                img = _request_capture()
                ok  = _do_solve(img)
                return '1' if ok else '0'
            except Exception as e:
                return 'err:%s' % e

        elif cmd == 'pause_solve':
            solve = False; return 'ok'

        elif cmd == 'resume_solve':
            solve = True;  return 'ok'

        elif cmd == 'set_offset':
            offset_cx, offset_cy = float(a), float(b)
            offset_str = 'cx=%.1f cy=%.1f' % (offset_cx, offset_cy)
            offset_flag.value = True
            return 'ok'

        elif cmd == 'clear_offset':
            offset_cx = offset_cy = 0.0
            offset_str = 'no offset'
            offset_flag.value = False
            return 'ok'

        elif cmd == 'auto_exp':
            img = _request_capture()
            exp = float(param.get('Exposure','0.2'))
            for _ in range(20):
                ext = tetra3rs.extract_centroids(img, sigma_threshold=10.0,
                                                 max_centroids=MAX_CENTROIDS)
                nc  = len(ext.centroids)
                pk  = _centroid_peak(ext.centroids, img)
                print('[solver] auto_exp: %d stars %d peak' % (nc, pk))
                if nc < 20:
                    exp *= 2
                elif nc > 50 and pk > 250:
                    exp = int((exp / 2) * 10) / 10
                else:
                    break
                _set_camera(exp, param.get('Gain','20'))
                img = _request_capture()
            return str(exp)

        elif cmd == 'go_solve':
            img = _request_capture()
            return '1' if _do_solve(img) else '0'

        elif cmd == 'measure_offset':
            offset_flag.value = True
            img = _request_capture()
            ok = _do_solve(img)
            if not ok:
                offset_flag.value = False; return 'fail'
            exp = float(param.get('Exposure', '0.2'))
            ext = tetra3rs.extract_centroids(img, sigma_threshold=_detect_sigma,
                                             max_centroids=MAX_CENTROIDS)
            peak = _centroid_peak(ext.centroids, img)
            while peak < 200 and exp < 4.0:
                exp = min(4.0, exp * 1.5)
                _set_camera(exp, param.get('Gain', '20'))
                img = _request_capture()
                ok = _do_solve(img)
                if not ok:
                    offset_flag.value = False; return 'fail'
                ext = tetra3rs.extract_centroids(img, sigma_threshold=_detect_sigma,
                                                 max_centroids=MAX_CENTROIDS)
                peak = _centroid_peak(ext.centroids, img)
            return offset_str

        elif cmd == 'shutdown':
            keep.value = False; return 'ok'

        return 'unknown_cmd'

    # ── live JPEG writer thread ──
    _live_q    = None
    _live_t    = None
    _QueueFull = None
    LIVE_IMAGE = os.path.join(home_path, 'Solver/images/live.jpg')
    fnt        = None
    try:
        from PIL import Image, ImageDraw, ImageEnhance, ImageFont, ImageOps
        from queue import Queue as _Queue, Full as _QueueFull, Empty as _QueueEmpty
        try:
            fnt = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf', 14)
        except Exception:
            fnt = ImageFont.load_default()
        _live_q = _Queue(maxsize=2)

        def _live_writer_thread():
            while True:
                try:
                    arr, overlay = _live_q.get()
                except Exception:
                    continue
                if arr is None:
                    continue
                try:
                    img  = Image.fromarray(arr)
                    img2 = ImageEnhance.Contrast(img).enhance(5)
                    if overlay is not None:
                        d = ImageDraw.Draw(img2)
                        d.text((5, 5), overlay, font=fnt, fill='white')
                    tmp = LIVE_IMAGE + '.tmp'
                    img2.save(tmp, format='JPEG')
                    os.replace(tmp, LIVE_IMAGE)
                except Exception:
                    pass

        _live_t = threading.Thread(target=_live_writer_thread, daemon=True)
        _live_t.start()
    except ImportError:
        pass

    def _save_debug(arr, txt):
        nonlocal frame_n, keep
        frame_n += 1
        img  = Image.fromarray(arr)
        img2 = ImageEnhance.Contrast(img).enhance(5)
        d    = ImageDraw.Draw(img2)
        d.text((70, 5), txt + '      Frame %d' % frame_n, font=fnt, fill='white')
        img2 = ImageOps.expand(img2, border=5, fill='red')
        img2.save(os.path.join(home_path, 'Solver/images/capture.jpg'))
        if frame_n > 1100:
            keep.value = False

    # ── maint Unix socket server thread ──
    MAINT_SOCK_PATH = '/run/efinder/maint.sock'

    def _dispatch_maint(cmd, args):
        """Translate webui maint commands to solver internals."""
        nonlocal _fov_measured, solve, solved_radec

        if cmd == 'ping':
            return {'pong': True}

        elif cmd == 'version':
            return {'version': '1.0.0-tetra3rs_mp'}

        elif cmd == 'status':
            try:
                with open(result_path) as _f:
                    sol = json.load(_f)
            except Exception:
                sol = None
            bs_x = FRAME_W / 2 + offset_cx
            bs_y = FRAME_H / 2 + offset_cy
            imu_n       = shared_cfg.get('imu_calib_n', 0)
            imu_quality = shared_cfg.get('imu_calib_quality', 0.0)
            imu_avail   = shared_cfg.get('imu_available', False)
            return {
                'solution':  sol,
                'boresight': {'x': bs_x, 'y': bs_y},
                'fov_deg':   _fov_measured,
                'solving':   solve,
                'imu': {
                    'available': imu_avail,
                    'calib_n':   imu_n,
                    'quality':   round(imu_quality, 3),
                    'active':    imu_avail and imu_n >= 3 and imu_quality >= 0.85,
                },
            }

        elif cmd == 'calibration_status':
            return _calibrator.get_status()

        elif cmd == 'calibration_reset':
            _calibrator.force_recalibrate()
            return {'reset': True}

        elif cmd == 'reset_offset':
            return _handle('clear_offset', '', '')

        elif cmd == 'boresight_show':
            return {'x': FRAME_W / 2 + offset_cx, 'y': FRAME_H / 2 + offset_cy}

        elif cmd == 'boresight_center':
            return _handle('clear_offset', '', '')

        elif cmd == 'exposure_get':
            return {
                'exposure_s': float(param.get('Exposure', '0.2')),
                'gain':       float(param.get('Gain', '20')),
            }

        elif cmd == 'exposure_set':
            s = float(args.get('exposure_s', param.get('Exposure', '0.2')))
            persist = bool(args.get('persist', True))
            _set_camera(s, float(param.get('Gain', '20')), persist=persist)
            return {'exposure_s': s}

        elif cmd == 'gain_set':
            g = float(args.get('gain', param.get('Gain', '20')))
            persist = bool(args.get('persist', True))
            _set_camera(float(param.get('Exposure', '0.2')), g, persist=persist)
            return {'gain': g}

        elif cmd == 'solver_params_get':
            return dict(param)

        elif cmd == 'solver_params_set':
            for k, v in args.items():
                param[k] = str(v)
            save_param(param)
            return {'ok': True}

        elif cmd == 'polar_start':
            _polar.start()
            return _polar.get_status()

        elif cmd == 'polar_status':
            return _polar.get_status()

        elif cmd == 'polar_cancel':
            _polar.cancel()
            return {'ok': True}

        elif cmd == 'polar_set_latitude':
            lat = float(args.get('latitude_deg', 0.0))
            param['Latitude'] = str(lat)
            save_param(param)
            _polar.set_latitude(lat)
            return {'latitude_deg': lat}

        else:
            raise ValueError('unknown maint command: %s' % cmd)

    def _maint_client_thread(conn):
        """Handle one maint connection: newline-delimited JSON in/out."""
        buf = b''
        try:
            while True:
                chunk = conn.recv(4096)
                if not chunk:
                    break
                buf += chunk
                while b'\n' in buf:
                    line, _, buf = buf.partition(b'\n')
                    if not line.strip():
                        continue
                    try:
                        req    = json.loads(line.decode('utf-8'))
                        cmd    = str(req.get('cmd', ''))
                        args   = req.get('args') or {}
                        result = _dispatch_maint(cmd, args)
                        resp   = json.dumps({'ok': True, 'result': result}) + '\n'
                    except Exception as e:
                        resp = json.dumps({'ok': False, 'error': str(e)}) + '\n'
                    try:
                        conn.sendall(resp.encode('utf-8'))
                    except Exception:
                        break
        except Exception:
            pass
        finally:
            try:
                conn.close()
            except Exception:
                pass

    def _maint_server_thread():
        """Bind Unix socket at MAINT_SOCK_PATH and accept connections."""
        sock_dir = os.path.dirname(MAINT_SOCK_PATH)
        try:
            os.makedirs(sock_dir, exist_ok=True)
        except Exception as e:
            log.warning('[solver] maint: could not create %s: %s', sock_dir, e)
            return
        try:
            os.unlink(MAINT_SOCK_PATH)
        except FileNotFoundError:
            pass
        except Exception as e:
            log.warning('[solver] maint: could not unlink stale socket: %s', e)
        try:
            srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            srv.bind(MAINT_SOCK_PATH)
            srv.listen(5)
            srv.settimeout(1.0)
            log.info('[solver] maint socket listening at %s', MAINT_SOCK_PATH)
        except Exception as e:
            log.error('[solver] maint: failed to bind socket: %s', e)
            return
        while keep.value:
            try:
                conn, _ = srv.accept()
                threading.Thread(target=_maint_client_thread, args=(conn,),
                                 daemon=True).start()
            except socket.timeout:
                pass
            except Exception as e:
                log.warning('[solver] maint accept error: %s', e)
        try:
            srv.close()
            os.unlink(MAINT_SOCK_PATH)
        except Exception:
            pass

    _maint_t = threading.Thread(target=_maint_server_thread, daemon=True)
    _maint_t.start()

    # ── main solve loop ──
    log.info('[solver] started')
    while keep.value:
        # service command queue (non-blocking)
        try:
            item = cmd_q.get_nowait()
            cmd, a, b = item if len(item)==3 else (*item, '', '')
            resp = _handle(cmd, a, b)
            result_q.put(resp)
        except Exception:
            pass

        if not solve:
            time.sleep(0.1); continue

        try:
            if not frame_ready.wait(timeout=1.0):
                continue
            frame_ready.clear()
            img = frame.copy()
            frame_n += 1
            ok = _do_solve(img)
            if ok and _polar.state not in (PolarState.IDLE, PolarState.DONE, PolarState.ERROR):
                _polar.update_from_solve(shared_ra.value, shared_dec.value)
        except Exception as e:
            log.warning('[solver] loop error: %s', e)

    shm.close()
    log.info('[solver] exiting')


# ───────────────────────────────────────────────────────────────────────────────────
# PROCESS 3 — lx200
# ───────────────────────────────────────────────────────────────────────────────────

def lx200_worker(
    frame_shm_name, frame_ready,
    shared_ra, shared_dec, offset_flag,
    keep, test_mode,
    cmd_q, result_q, cam_cmd_q,
    shared_cfg,
    *_extra,
):
    import signal, socket
    signal.signal(signal.SIGINT, signal.SIG_IGN)

    param = load_param()
    host  = param.get('Host', '0.0.0.0')
    port  = int(param.get('Port', '4030'))

    # pending command state
    _pending_cmd  = None
    _pending_lock = threading.Lock()
    _result_ready = threading.Event()
    _last_result  = [None]

    def _send_cmd(cmd, a='', b=''):
        """Send command to solver; return response (blocks up to 2 s)."""
        _result_ready.clear()
        cmd_q.put((cmd, a, b))
        _result_ready.wait(timeout=2.0)
        return _last_result[0]

    def _result_poller():
        while keep.value:
            try:
                r = result_q.get(timeout=0.5)
                _last_result[0] = r
                _result_ready.set()
            except Exception:
                pass

    threading.Thread(target=_result_poller, daemon=True).start()

    # ── LX200 parser ──

    def _ra_to_lx200(ra_deg):
        h = ra_deg / 15.0
        hh = int(h)
        mm = int((h - hh) * 60)
        ss = int(((h - hh) * 60 - mm) * 60)
        return '%02d:%02d:%02d' % (hh, mm, ss)

    def _dec_to_lx200(dec_deg):
        sign = '+' if dec_deg >= 0 else '-'
        d = abs(dec_deg)
        dd = int(d)
        mm = int((d - dd) * 60)
        ss = int(((d - dd) * 60 - mm) * 60)
        return '%s%02d*%02d:%02d' % (sign, dd, mm, ss)

    def _handle_lx200(data, conn):
        """Process one LX200 command string; send response via conn."""
        data = data.strip()
        if not data: return

        def send(s):
            try: conn.sendall(s.encode())
            except Exception: pass

        # :GR# — get RA (IMU dead-reckoning with fallback)
        if data == ':GR#':
            pred = _imu_predict(shared_cfg)
            if pred is not None:
                send(_ra_to_lx200(pred[0]) + '#')
            else:
                send(_ra_to_lx200(shared_ra.value) + '#')

        # :GD# — get Dec (IMU dead-reckoning with fallback)
        elif data == ':GD#':
            pred = _imu_predict(shared_cfg)
            if pred is not None:
                send(_dec_to_lx200(pred[1]) + '#')
            else:
                send(_dec_to_lx200(shared_dec.value) + '#')

        # :GS# — get sidereal time (dummy)
        elif data == ':GS#':
            t  = datetime.datetime.utcnow()
            hh = (t.hour + t.minute/60 + t.second/3600) % 24
            send(_ra_to_lx200(hh * 15) + '#')

        # :SC# — sync / set date (ignore, acknowledge)
        elif data.startswith(':SC'):
            send('1Updating        #')

        # :CM# — sync on current position
        elif data == ':CM#':
            _send_cmd('solve_now')
            send('Synced          #')

        # :Q# — stop (no-op for plate solver)
        elif data == ':Q#':
            pass

        # :MS# — slew (no-op)
        elif data == ':MS#':
            send('0')

        # :GW# — mount status (required for SkySafari init handshake)
        elif data == ':GW#':
            send('AT2#')

        # :GVP#/:GVN#/:GVF#/:GVD#/:GVT# — firmware version queries
        elif data.startswith(':GV'):
            send('eFinder-tetra3rs#')

        # :GT# — tracking rate
        elif data == ':GT#':
            send('60.0#')

        # :Gr# — slew rate
        elif data == ':Gr#':
            send('4#')

        # :GA# — altitude (not tracked, return zero)
        elif data == ':GA#':
            send('+00*00#')

        # :GZ# — azimuth
        elif data == ':GZ#':
            send('000*00#')

        # :GC# — date
        elif data == ':GC#':
            t = datetime.datetime.utcnow()
            send('%02d/%02d/%02d#' % (t.month, t.day, t.year % 100))

        # :GL# — local time
        elif data == ':GL#':
            t = datetime.datetime.utcnow()
            send('%02d:%02d:%02d#' % (t.hour, t.minute, t.second))

        # :GG# — UTC offset
        elif data == ':GG#':
            send('+0.0#')

        # :Gt# — site latitude (return stored value or 0)
        elif data == ':Gt#':
            lat = param.get('Latitude', '0.0')
            deg = float(lat)
            sign = '+' if deg >= 0 else '-'
            d = abs(deg)
            send('%s%02d*%02d#' % (sign, int(d), int((d % 1) * 60)))

        # :Gg# — site longitude
        elif data == ':Gg#':
            lon = param.get('Longitude', '0.0')
            deg = float(lon)
            sign = '+' if deg >= 0 else '-'
            d = abs(deg)
            send('%s%03d*%02d#' % (sign, int(d), int((d % 1) * 60)))

        # :St# — set site latitude
        elif data.startswith(':St'):
            param['Latitude'] = data[3:].rstrip('#')
            send('1')

        # :Sg# — set site longitude
        elif data.startswith(':Sg'):
            param['Longitude'] = data[3:].rstrip('#')
            send('1')

        # :SL# — set local time (acknowledge, ignore)
        elif data.startswith(':SL'):
            send('1')

        # :SG# — set UTC offset (acknowledge, ignore)
        elif data.startswith(':SG'):
            send('1')

        # :Sr# — set target RA (acknowledge)
        elif data.startswith(':Sr'):
            send('1')

        # :Sd# — set target Dec (acknowledge)
        elif data.startswith(':Sd'):
            send('1')

        # :MA#/:Me#/:Mw#/:Mn#/:Ms# — motion commands (no-op)
        elif data.startswith(':M') and data != ':MS#':
            send('')

        # :Re#/:Rw#/:Rn#/:Rs# — rate commands (acknowledge)
        elif data.startswith(':R'):
            send('')

        # efinder extensions via :X<cmd> #
        elif data.startswith(':X'):
            inner = data[2:].rstrip('#').strip()
            parts = inner.split(None, 2)
            cmd   = parts[0] if parts else ''
            a     = parts[1] if len(parts) > 1 else ''
            b     = parts[2] if len(parts) > 2 else ''
            resp  = _send_cmd(cmd, a, b)
            send((resp or '') + '#')

        else:
            log.debug('[lx200] unknown cmd: %r', data)

    def _client_thread(conn, addr):
        log.info('[lx200] connect %s', addr)
        try:
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except Exception:
            pass
        buf = ''
        try:
            while keep.value:
                chunk = conn.recv(256)
                if not chunk: break
                buf += chunk.decode(errors='replace')
                while '#' in buf:
                    cmd, _, buf = buf.partition('#')
                    _handle_lx200(cmd + '#', conn)
        except Exception as e:
            log.debug('[lx200] client error: %s', e)
        finally:
            conn.close()
        log.info('[lx200] disconnect %s', addr)

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    srv.bind((host, port))
    srv.listen(5)
    srv.settimeout(1.0)
    log.info('[lx200] listening on %s:%d', host, port)

    while keep.value:
        try:
            conn, addr = srv.accept()
            threading.Thread(target=_client_thread, args=(conn, addr),
                             daemon=True).start()
        except socket.timeout:
            pass
        except Exception as e:
            log.warning('[lx200] accept error: %s', e)

    srv.close()
    log.info('[lx200] exiting')


# ───────────────────────────────────────────────────────────────────────────────────
# PROCESS 0 — main / supervisor
# ───────────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='eFinder tetra3rs multiprocess')
    parser.add_argument('--test', action='store_true', help='Synthetic camera (no Pi)')
    parser.add_argument('--no-update', action='store_true', help='Skip OTA check')
    args = parser.parse_args()

    log.info('[main] starting, home=%s', home_path)

    # OTA
    if not args.no_update:
        ver, url = check_update()
        if ver:
            log.info('[main] update available: %s', ver)
            try:
                apply_update(url)
                log.info('[main] restarting after update')
                os.execv(sys.executable, [sys.executable] + sys.argv)
            except Exception as e:
                log.warning('[main] update failed: %s', e)

    # shared memory
    shm = shared_memory.SharedMemory(create=True, size=FRAME_BYTES)

    # shared state
    shared_ra   = mp.Value(c_double, 0.0)
    shared_dec  = mp.Value(c_double, 0.0)
    offset_flag = mp.Value(c_bool,   False)
    keep        = mp.Value(c_bool,   True)
    test_mode   = mp.Value(c_bool,   args.test)

    # queues
    cmd_q     = mp.Queue()
    result_q  = mp.Queue()
    cam_cmd_q = mp.Queue()

    # event
    frame_ready = mp.Event()

    # IMU shared state (manager dict — accessible from all worker processes)
    mgr = mp.Manager()
    shared_cfg = mgr.dict({
        'imu_available':    False,
        'imu_q':            None,
        'imu_t':            0.0,
        'imu_ref_q':        None,
        'imu_ref_ra_deg':   None,
        'imu_ref_dec_deg':  None,
        'imu_ref_t':        0.0,
        'imu_ref_roll_deg': 0.0,
        'imu_calib_C':      None,
        'imu_calib_n':      0,
        'imu_calib_quality': 0.0,
    })

    shared = (
        shm.name, frame_ready,
        shared_ra, shared_dec, offset_flag,
        keep, test_mode,
        cmd_q, result_q, cam_cmd_q,
        shared_cfg,
    )

    proc_specs = [
        {'name': 'camera', 'target': camera_worker},
        {'name': 'solver', 'target': solver_worker},
        {'name': 'lx200',  'target': lx200_worker},
    ]

    workers = {spec['name']: _start_worker(spec, shared) for spec in proc_specs}

    try:
        while keep.value:
            time.sleep(HEALTH_POLL)
            for spec in proc_specs:
                name = spec['name']
                p    = workers[name]
                if not p.is_alive():
                    cnt = restart_counts[name] + 1
                    restart_counts[name] = cnt
                    log.warning('[main] %s died (exit=%s), restart #%d',
                                name, p.exitcode, cnt)
                    if cnt > RESTART_LIMIT:
                        log.error('[main] %s exceeded restart limit, stopping', name)
                        keep.value = False
                        break
                    workers[name] = _start_worker(spec, shared)
    except KeyboardInterrupt:
        log.info('[main] KeyboardInterrupt — shutting down')
        keep.value = False

    # tidy up
    for name, p in workers.items():
        p.join(timeout=3)
        if p.is_alive():
            log.warning('[main] force-terminating %s', name)
            p.terminate()

    mgr.shutdown()
    shm.unlink()
    log.info('[main] done')


if __name__ == '__main__':
    mp.set_start_method('fork')
    main()
