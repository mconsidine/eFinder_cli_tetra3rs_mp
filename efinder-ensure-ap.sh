#!/bin/bash
# efinder-ensure-ap.sh — run at every boot to guarantee the eFinder AP is up.
#
# NetworkManager suppresses AP autoconnect after a failed activation (e.g.
# radio not ready at boot, or profile marked failed after station.sh was
# used). Once suppressed, NM won't retry across reboots without intervention.
#
# This script polls wlan0 for up to 60 s; if the interface is still idle
# it brings up efinder-ap unconditionally, clearing any NM autoconnect
# suppression flags first.
#
# Installed by install_tetra3rs_mp.sh; enabled via:
#   sudo systemctl enable efinder-ensure-ap.service
set -euo pipefail

AP_PROFILE="efinder-ap"
POLL_INTERVAL=5
MAX_POLLS=12   # 12 x 5s = 60 s

echo "[ensure-ap] checking wlan0 state (up to $((POLL_INTERVAL * MAX_POLLS))s)..."

for i in $(seq 1 "$MAX_POLLS"); do
    # Treat 'connected' (station) and 'activated' (AP) both as "something
    # is up" — if the user is in station mode we don't stomp on it.
    STATE=$(nmcli -t -f DEVICE,STATE device 2>/dev/null \
            | awk -F: '$1=="wlan0" {print $2; exit}')
    if [[ "$STATE" == "connected" || "$STATE" == "activated" ]]; then
        echo "[ensure-ap] wlan0 is $STATE — nothing to do."
        exit 0
    fi
    echo "[ensure-ap] attempt $i/$MAX_POLLS: wlan0='$STATE' — waiting ${POLL_INTERVAL}s..."
    sleep "$POLL_INTERVAL"
done

echo "[ensure-ap] wlan0 still idle after $((POLL_INTERVAL * MAX_POLLS))s — forcing AP up."

# Clear NM autoconnect suppression that may have accumulated from prior
# station-mode failures or incomplete activations.
nmcli con mod "$AP_PROFILE" \
    connection.autoconnect yes \
    connection.autoconnect-retries 0 2>/dev/null || true

if nmcli con up "$AP_PROFILE"; then
    echo "[ensure-ap] AP brought up successfully."
else
    echo "[ensure-ap] WARNING: 'nmcli con up $AP_PROFILE' failed."
    exit 1
fi
