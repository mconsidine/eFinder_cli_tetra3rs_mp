#!/bin/bash
# ap.sh — switch wlan0 back to eFinder AP mode.
set -euo pipefail
AP_PROFILE="efinder-ap"
nmcli con mod "$AP_PROFILE" connection.autoconnect yes connection.autoconnect-retries 0 2>/dev/null || true
if ! nmcli con up "$AP_PROFILE"; then exit 1; fi
while IFS=: read -r name type; do
    [ "$name" = "$AP_PROFILE" ] && continue
    [ "$type" = "802-11-wireless" ] || continue
    nmcli con mod "$name" connection.autoconnect no 2>/dev/null || true
done < <(nmcli -t -f NAME,TYPE con show)
