"""
FOV calibration state machine for tetra3rs_mp.
Adapted from efinder/calibration.py for the INI-based eFinder.config system.
"""
import statistics
from collections import deque
from dataclasses import dataclass
from enum import Enum
from typing import Optional


class CalState(Enum):
    UNCALIBRATED = "uncalibrated"
    CALIBRATING  = "calibrating"
    CALIBRATED   = "calibrated"


@dataclass
class CalibrationParams:
    window_size:                   int   = 30
    fov_convergence_stddev:        float = 0.05
    fov_drift_sigmas:              float = 3.0
    drift_check_interval:          int   = 50
    distortion_convergence_stddev: float = 0.005


class FovCalibrator:
    """
    Tracks rolling FOV and distortion measurements from tetra3rs and commits
    to eFinder.config once the window converges (stddev < threshold over
    window_size solves).

    State transitions:
      UNCALIBRATED -> CALIBRATING  (first solve)
      CALIBRATING  -> CALIBRATED   (window full AND FOV stddev converged)
      CALIBRATED   -> CALIBRATING  (drift detected: rolling median moved
                                    > fov_drift_sigmas from committed value)
    """

    def __init__(self, param: dict, save_param_fn,
                 fov_estimate: float, frame_width: int,
                 params: Optional[CalibrationParams] = None):
        self._param        = param
        self._save_param   = save_param_fn
        self._fov_estimate = fov_estimate
        self._frame_width  = frame_width
        self.params        = params or CalibrationParams()

        calibrated    = str(param.get('fov_calibrated', '0')).strip() == '1'
        committed_str = param.get('fov_measured', '0')
        try:
            committed_fov = float(committed_str) if committed_str else 0.0
        except ValueError:
            committed_fov = 0.0

        if calibrated and committed_fov > 0:
            self.state              = CalState.CALIBRATED
            self.committed_fov      = committed_fov
            try:
                self.committed_fov_stddev = float(param.get('fov_calibrated_stddev', '0.05'))
            except ValueError:
                self.committed_fov_stddev = 0.05
        else:
            self.state              = CalState.UNCALIBRATED
            self.committed_fov      = None
            self.committed_fov_stddev = None

        self._fov_window         = deque(maxlen=self.params.window_size)
        self._solves_since_check = 0

        try:
            self.committed_distortion = float(self._param.get('distortion', '0.0'))
        except ValueError:
            self.committed_distortion = 0.0
        self._distortion_window = deque(maxlen=self.params.window_size)

    # ---- Public API --------------------------------------------------------

    def get_fov_estimate(self) -> float:
        """Best current FOV estimate to pass to tetra3rs solve_from_centroids."""
        return self.committed_fov if self.committed_fov else self._fov_estimate

    def get_fov_max_error(self) -> float:
        """Tight tolerance once calibrated; wide while still converging."""
        return 0.3 if self.state == CalState.CALIBRATED else 1.0

    def get_distortion_estimate(self) -> float:
        """Best current distortion coefficient (0.0 until first convergence)."""
        return self.committed_distortion

    def update_from_solve(self, fov_deg: float, distortion: float = 0.0) -> None:
        """Called after every successful solve with the measured FOV and distortion."""
        if not fov_deg or fov_deg <= 0:
            return
        self._fov_window.append(fov_deg)
        self._distortion_window.append(distortion)

        if self.state == CalState.UNCALIBRATED:
            self.state = CalState.CALIBRATING

        if len(self._fov_window) < self.params.window_size:
            return

        if self.state == CalState.CALIBRATING:
            self._maybe_commit()
        elif self.state == CalState.CALIBRATED:
            self._solves_since_check += 1
            if self._solves_since_check >= self.params.drift_check_interval:
                self._solves_since_check = 0
                self._check_for_drift()

    def get_status(self) -> dict:
        """Snapshot suitable for the maintenance protocol and STATE_FILE."""
        recent_med = statistics.median(self._fov_window) if self._fov_window else None
        recent_std = (statistics.stdev(self._fov_window)
                      if len(self._fov_window) > 1 else None)
        return {
            'state':                self.state.value,
            'window_size':          self.params.window_size,
            'window_filled':        len(self._fov_window),
            'committed_fov':        round(self.committed_fov, 4) if self.committed_fov else None,
            'committed_fov_stddev': round(self.committed_fov_stddev, 4)
                                    if self.committed_fov_stddev else None,
            'committed_distortion': round(self.committed_distortion, 6),
            'recent_median':        round(recent_med, 4) if recent_med else None,
            'recent_stddev':        round(recent_std, 4) if recent_std else None,
            'current_max_error':    self.get_fov_max_error(),
        }

    def force_recalibrate(self) -> None:
        """Discard committed state and restart measurement from scratch."""
        self._fov_window.clear()
        self._distortion_window.clear()
        self.committed_fov        = None
        self.committed_fov_stddev = None
        self.committed_distortion = 0.0
        self.state                = CalState.UNCALIBRATED
        self._solves_since_check  = 0
        self._param['fov_calibrated']        = '0'
        self._param['fov_calibrated_stddev'] = '%.4f' % self.params.fov_convergence_stddev
        self._param['distortion']            = '0.000000'
        try:
            self._save_param(self._param)
        except Exception as e:
            print('[calibration] could not persist reset: %s' % e)
        print('[calibration] force recalibrate: state -> uncalibrated')

    # ---- Internal ----------------------------------------------------------

    def _maybe_commit(self) -> None:
        fov_med = statistics.median(self._fov_window)
        fov_std = (statistics.stdev(self._fov_window)
                   if len(self._fov_window) > 1 else 0.0)

        if fov_std > self.params.fov_convergence_stddev:
            return

        arcsec_per_pixel = fov_med * 3600.0 / self._frame_width
        self._param['fov_measured']          = '%.4f' % fov_med
        self._param['fov_calibrated']        = '1'
        self._param['fov_calibrated_stddev'] = '%.4f' % fov_std
        self._param['arcsec_per_pixel']      = '%.4f' % arcsec_per_pixel

        dist_committed = False
        if len(self._distortion_window) >= self.params.window_size:
            dist_med = statistics.median(self._distortion_window)
            dist_std = (statistics.stdev(self._distortion_window)
                        if len(self._distortion_window) > 1 else 0.0)
            if dist_std <= self.params.distortion_convergence_stddev:
                self._param['distortion'] = '%.6f' % dist_med
                self.committed_distortion = dist_med
                dist_committed = True

        try:
            self._save_param(self._param)
        except Exception as e:
            print('[calibration] could not persist FOV: %s' % e)
            return

        self.committed_fov        = fov_med
        self.committed_fov_stddev = fov_std
        self.state                = CalState.CALIBRATED
        print('[calibration] FOV calibrated: %.4f deg +/- %.4f (n=%d)%s' % (
            fov_med, fov_std, len(self._fov_window),
            '' if not dist_committed else '; distortion=%.4f' % self.committed_distortion))

    def _check_for_drift(self) -> None:
        if self.committed_fov is None or self.committed_fov_stddev is None:
            return
        recent_med = statistics.median(self._fov_window)
        scale = max(self.committed_fov_stddev, self.params.fov_convergence_stddev)
        drift = abs(recent_med - self.committed_fov) / scale
        if drift > self.params.fov_drift_sigmas:
            print('[calibration] FOV drift: was %.4f now %.4f (%.1f sigma); recalibrating' % (
                self.committed_fov, recent_med, drift))
            self.state = CalState.CALIBRATING
            self._solves_since_check = 0
