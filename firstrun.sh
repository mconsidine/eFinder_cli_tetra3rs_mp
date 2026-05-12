#!/bin/bash
# firstrun.sh — staged into the image at build time, runs once on first boot
# (gated by /home/efinder/.run_firstrun).
#
# The CI image already pre-stages Solver/, the tetra3rs wheel and the venv.
# This script is the manual-rebuild path: when a user touches
# /home/efinder/.run_firstrun and reboots, we re-clone the repo (master copy)
# over /home/efinder/Solver. Useful for development or recovering a
# corrupted Solver/ tree without re-flashing.
set -e
LOG=/home/efinder/firstrun.log
exec > >(tee -a "$LOG") 2>&1

echo "=== eFinder first-boot setup: $(date) ==="

# Wait for internet (needed to clone repo and run apt)
echo "Waiting for internet connectivity..."
connected=false
for i in $(seq 1 30); do
    if ping -c1 -W2 github.com &>/dev/null; then
        connected=true
        echo "  Internet reachable after attempt $i"
        break
    fi
    echo "  attempt $i/30 -- retrying in 10s..."
    sleep 10
done

if [ "$connected" != "true" ]; then
    echo "ERROR: No internet after 5 minutes -- aborting first-boot setup."
    echo "       Ensure the Pi has WiFi or USB ethernet on first boot."
    exit 1
fi

# Fix ownership now that the efinder user exists at runtime
chown -R efinder:efinder /home/efinder

# Clone the correct repo (dev branch, matching the image build pipeline)
REPO_URL=https://github.com/mconsidine/eFinder_cli_tetra3rs_mp.git
REPO_BRANCH=dev
REPO_DIR=/home/efinder/eFinder_cli_tetra3rs_mp

cd /home/efinder
if [ -d "$REPO_DIR" ]; then
    echo "Repo already present -- pulling latest..."
    git -C "$REPO_DIR" fetch origin
    git -C "$REPO_DIR" reset --hard "origin/${REPO_BRANCH}"
else
    git clone --branch "$REPO_BRANCH" "$REPO_URL" "$REPO_DIR"
fi
chown -R efinder:efinder "$REPO_DIR"

# Refresh Solver/ from the freshly-cloned tree (preserves images/, uploads/,
# eFinder.config edits made by the user via the web UI).
if [ -d "$REPO_DIR/Solver" ]; then
    rsync -a --exclude='images/' --exclude='eFinder.config' \
        "$REPO_DIR/Solver/" /home/efinder/Solver/
    chown -R efinder:efinder /home/efinder/Solver
fi

# Remove trigger file so firstrun.service skips on all subsequent boots.
# To re-run: touch /home/efinder/.run_firstrun && sudo reboot
rm -f /home/efinder/.run_firstrun

echo "=== eFinder first-boot setup complete: $(date) ==="
reboot
