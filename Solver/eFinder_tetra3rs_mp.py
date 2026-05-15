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
#   cam_cmd_q      — Queue  solver -> camera: set_exp, captur
#   cam_result_q   — Queue  camera -> solver: captured frames
#
# /dev/shm/efinder_state.json  written by solver after each solve — slow
#   telemetry (stars, peak, eTime, etc.) read by lx200 only for diagnostic
#   commands (:GS, :GK, :GE) which are not timing-critical.
#   :Gt / :Gg now return SkySafari-format lat/lon (set via :St/:Sg);
#   per-solve elapsed seconds moved from :Gt to :GE.
