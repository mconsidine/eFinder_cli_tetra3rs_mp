#!/bin/bash
AP_PROFILE="efinder-ap"
POLL_INTERVAL=5
MAX_POLLS=12
for i in $(seq 1 "$MAX_POLLS"); do
    STATE=$(nmcli -t -f DEVICE,STATE device 2>/dev/null | grep "^wlan0:" | cut -d: -f2)
    if [[ "$STATE" == "connected" || "$STATE" == "activated" ]]; then
        exit 0
    fi
    sleep "$POLL_INTERVAL"
done
nmcli con mod "$AP_PROFILE" connection.autoconnect yes connection.autoconnect-retries 0 2>/dev/null || true
nmcli con up "$AP_PROFILE"
