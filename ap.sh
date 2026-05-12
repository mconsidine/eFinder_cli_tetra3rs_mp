#!/bin/bash
# ap.sh — switch wlan0 back to eFinder AP mode.
#
# Run after station.sh to return to AP-only mode, or call directly
# when the Pi cannot be reached via the home LAN.
#
# Usage: ~/ap.sh   (runs as efinder; uses nmcli which requires no sudo)
set -euo pipefail

AP_PROFILE="efinder-ap"

echo "Switching to AP mode..."

# Clear any autoconnect suppression that station.sh may have left behind.
# Without this, nmcli con up succeeds now but NM may not auto-reconnect
# on the next boot if the suppression counter is still set.
nmcli con mod "$AP_PROFILE" \
    connection.autoconnect yes \
    connection.autoconnect-retries 0 2>/dev/null || true

if ! nmcli con up "$AP_PROFILE"; then
    echo "ERROR: Failed to bring up $AP_PROFILE."
    exit 1
fi

# Mark any station-mode (non-AP) Wi-Fi profiles as non-autoconnecting
# so they don't compete with the AP on the next boot.
while IFS=: read -r name type; do
    [ "$name" = "$AP_PROFILE" ] && continue
    [ "$type" = "802-11-wireless" ] || continue
    nmcli con mod "$name" connection.autoconnect no 2>/dev/null || true
    echo "  disabled autoconnect for: $name"
done < <(nmcli -t -f NAME,TYPE con show)

SSID=$(nmcli -t -f 802-11-wireless.ssid con show "$AP_PROFILE" 2>/dev/null \
       | cut -d: -f2 || echo "efinder-ap")

echo ""
echo "AP mode active."
echo "  SSID : $SSID"
echo "  IP   : 192.168.50.1"
echo "  SSH  : ssh efinder@192.168.50.1"
echo ""
echo "Run ~/station.sh to reconnect to a home network."
