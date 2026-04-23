#!/bin/bash
# =============================================================================
# eFinder tetra3rs_mp Install Script for Raspberry Pi Zero 2W
# Combines the tetra3rs Rust solver with tiny_img's multiprocess architecture.
# Adds OTA update infrastructure and richer image-slim step from tiny_img.
# No gRPC, no cedar-detect-server, no cedar-solve, no Hipparcos download.
#
# Usage:
#   Interactive : sudo bash install_tetra3rs_mp.sh
#   Automated   : bash install_tetra3rs_mp.sh --non-interactive
#                 (WIFI_PASS and SAMBA_PASS must be exported by caller)
# =============================================================================
set -eo pipefail

EFINDER_HOME=/home/efinder
EFINDER_USER=efinder
VENV="$EFINDER_HOME/venv-efinder"
INSTALL_MARKER="$EFINDER_HOME/.efinder_installed"
APP_SCRIPT="eFinder_tetra3rs_mp.py"
NON_INTERACTIVE=false
[[ "$1" == "--non-interactive" ]] && NON_INTERACTIVE=true

# ---------------------------------------------------------------------------
# Guard: must run as efinder user (with sudo)
# ---------------------------------------------------------------------------
if [ "$NON_INTERACTIVE" = false ]; then
    RUNNING_AS="${SUDO_USER:-$(logname 2>/dev/null)}"
    if [ "$RUNNING_AS" != "$EFINDER_USER" ]; then
        echo "ERROR: Run as '$EFINDER_USER': sudo bash install_tetra3rs_mp.sh"
        exit 1
    fi
fi

# ---------------------------------------------------------------------------
# Guard: hardware model
# ---------------------------------------------------------------------------
PI_MODEL=$(tr -d '\0' < /proc/device-tree/model 2>/dev/null || echo "unknown")
if [[ "$PI_MODEL" != *"Zero 2"* ]]; then
    echo "WARNING: Designed for Raspberry Pi Zero 2W. Detected: $PI_MODEL"
    if [ "$NON_INTERACTIVE" = false ]; then
        read -rp "Continue anyway? [y/N] " confirm
        [[ "$confirm" =~ ^[Yy]$ ]] || exit 1
    fi
fi

# ---------------------------------------------------------------------------
# Guard: re-run check
# ---------------------------------------------------------------------------
if [ -f "$INSTALL_MARKER" ] && [ "$NON_INTERACTIVE" = false ]; then
    echo "NOTE: Previously installed. Re-run full install?"
    read -rp "[y/N] " rerun
    [[ "$rerun" =~ ^[Yy]$ ]] || exit 0
fi

echo "============================================================================="
echo " eFinder tetra3rs_mp Installer"
echo " Device : $PI_MODEL"
echo " Mode   : $( [ "$NON_INTERACTIVE" = true ] && echo non-interactive || echo interactive )"
echo "============================================================================="

sudo -v
while true; do sudo -n true; sleep 50; kill -0 "$$" || exit; done 2>/dev/null &
SUDO_KEEPALIVE_PID=$!
trap 'kill $SUDO_KEEPALIVE_PID 2>/dev/null' EXIT

# ---------------------------------------------------------------------------
# Credentials
# ---------------------------------------------------------------------------
if [ "$NON_INTERACTIVE" = true ]; then
    : "${WIFI_PASS:?ERROR: WIFI_PASS must be exported}"
    : "${SAMBA_PASS:?ERROR: SAMBA_PASS must be exported}"
else
    read -rsp "WiFi AP Password [12345678]: " WIFI_PASS;  echo
    WIFI_PASS="${WIFI_PASS:-12345678}"
    read -rsp "Samba Password   [12345678]: " SAMBA_PASS; echo
    SAMBA_PASS="${SAMBA_PASS:-12345678}"
fi

# ---------------------------------------------------------------------------
# 1. System update & packages
# ---------------------------------------------------------------------------
echo ""
echo "[0/9] Waiting for NTP clock sync..."
sudo systemctl restart systemd-timesyncd
for i in $(seq 1 12); do
    if timedatectl status 2>/dev/null | grep -q "System clock synchronized: yes"; then
        echo "  Clock synced."; break
    fi
    echo "  Waiting... ($i/12)"; sleep 5
done

# ---------------------------------------------------------------------------
# Time zone and locale (configured before apt so the upgrade's locale-gen
# step produces the right one, avoiding the cosmetically confusing
# "Generating locales... en_GB.UTF-8" line in the install log).
#
# Base image ships with Europe/London + en_GB.UTF-8 (upstream's origin).
# Default to UTC + en_US.UTF-8 here. UTC is convention for astronomy logs
# and matches what SkySafari sends over LX200 :SC# (which sets system UTC
# directly). en_US.UTF-8 gives familiar number/date formatting for US users.
#
# Override via env vars (use -E with sudo to preserve them):
#     TIMEZONE="Europe/Paris"  LOCALE="fr_FR.UTF-8"  sudo -E bash install.sh
# Use `timedatectl list-timezones` to see valid TZ names.
# ---------------------------------------------------------------------------
TIMEZONE="${TIMEZONE:-UTC}"
LOCALE="${LOCALE:-en_US.UTF-8}"

echo ""
echo "[0/9] Configuring locale and timezone..."
echo "  Time zone: $TIMEZONE"
if [ -f "/usr/share/zoneinfo/$TIMEZONE" ]; then
    sudo timedatectl set-timezone "$TIMEZONE" 2>/dev/null || \
        sudo ln -sf "/usr/share/zoneinfo/$TIMEZONE" /etc/localtime
    echo "$TIMEZONE" | sudo tee /etc/timezone > /dev/null
else
    echo "  WARNING: timezone '$TIMEZONE' not found, leaving system default"
fi

echo "  Locale: $LOCALE"
# Uncomment the requested locale in /etc/locale.gen if it isn't already
# available, then regenerate. Idempotent.
if ! locale -a 2>/dev/null | grep -qi "^${LOCALE//-/}\$\|^${LOCALE}\$"; then
    sudo sed -i "s/^# *\($(echo "$LOCALE" | sed 's/\./\\./g') .*\)/\1/" /etc/locale.gen 2>/dev/null || true
    sudo locale-gen "$LOCALE" 2>/dev/null || true
fi
sudo update-locale LANG="$LOCALE" LC_ALL="$LOCALE" 2>/dev/null || true

echo ""
echo "[1/9] Updating system packages..."
sudo apt-get update -q
sudo apt-get upgrade -y

echo ""
echo "[2/9] Installing required packages..."
sudo apt-get install -y --no-install-recommends \
    git \
    python3-pip \
    python3-numpy \
    python3-pillow \
    python3-picamera2 \
    python3-scipy \
    libopenblas-dev \
    samba \
    samba-common-bin \
    apache2 \
    php \
    libapache2-mod-php

# ---------------------------------------------------------------------------
# 2. Python virtual environment
# ---------------------------------------------------------------------------
echo ""
echo "[3/9] Setting up Python virtual environment..."
sudo -u "$EFINDER_USER" python3 -m venv "$VENV" --system-site-packages
"$VENV/bin/pip" install --upgrade pip
"$VENV/bin/pip" install --prefer-binary "Pillow>=9.0"
"$VENV/bin/pip" install --prefer-binary pyserial

# gaia-catalog: bundled Gaia DR3 + Hipparcos star data used by tetra3rs's
# generate_from_gaia(). It *should* be an auto-dependency of tetra3_python,
# but installing from a local wheel file skips dependency resolution, so
# we install it explicitly to make the offline-install path work too.
"$VENV/bin/pip" install --prefer-binary gaia-catalog

# tetra3rs — use staged aarch64 wheel (built by CI with Cortex-A53 + NEON),
# fall back to PyPI.
# Distribution name on PyPI is 'tetra3_python' (the Python bindings for
# the 'tetra3' Rust crate, which imports as `tetra3rs`). The project
# publishes the code under one name and the dist-info under another.
TETRA3RS_SRC="$EFINDER_HOME/tetra3rs-src"
TETRA3RS_WHEEL=$(ls "$TETRA3RS_SRC"/*.whl 2>/dev/null | head -1)
if [ -n "$TETRA3RS_WHEEL" ]; then
    echo "  Installing tetra3rs from staged wheel: $TETRA3RS_WHEEL"
    "$VENV/bin/pip" install "$TETRA3RS_WHEEL"
else
    echo "  Installing tetra3_python from PyPI (pre-built ARM64 wheel)..."
    "$VENV/bin/pip" install --prefer-binary tetra3_python
fi

# ---------------------------------------------------------------------------
# Workaround for upstream packaging bug (as of tetra3_python 0.4.1–0.6.0+):
#   tetra3rs/__init__.py calls importlib.metadata.version("tetra3rs")
# but the dist-info installed by pip is named `tetra3_python-*.dist-info`,
# so the lookup raises PackageNotFoundError at import time, crashing
# everything before the first solve.
#
# We cannot use `import tetra3rs` to find __init__.py — the import is
# what's broken. Instead, ask pip directly via its sysconfig-based
# site-packages path. This also survives future Python versions where
# the path contains 'python3.14', etc.
# ---------------------------------------------------------------------------
SITE_PACKAGES=$("$VENV/bin/python3" -c \
    "import sysconfig; print(sysconfig.get_paths()['purelib'])" 2>/dev/null || true)
TETRA3RS_INIT=""
if [ -n "$SITE_PACKAGES" ] && [ -f "$SITE_PACKAGES/tetra3rs/__init__.py" ]; then
    TETRA3RS_INIT="$SITE_PACKAGES/tetra3rs/__init__.py"
fi

if [ -n "$TETRA3RS_INIT" ]; then
    if grep -q 'version("tetra3rs")' "$TETRA3RS_INIT"; then
        echo "  Patching $TETRA3RS_INIT (upstream name-mismatch bug)..."
        sudo sed -i 's/version("tetra3rs")/version("tetra3_python")/' "$TETRA3RS_INIT"
    else
        echo "  tetra3rs/__init__.py already patched or upstream bug is fixed — no action."
    fi
else
    echo "  WARNING: could not locate tetra3rs/__init__.py; if the import"
    echo "  check below fails, patch it manually."
fi

# Verify the fix: if any of these imports fail, the database generation
# step ahead will fail too — catch them here with clear messages instead.
if ! "$VENV/bin/python3" -c "import tetra3rs; _ = tetra3rs.__version__" 2>/dev/null; then
    echo "ERROR: tetra3rs import check failed after patch. Investigate with:"
    echo "       $VENV/bin/python3 -c 'import tetra3rs'"
    exit 1
fi
if ! "$VENV/bin/python3" -c "import gaia_catalog" 2>/dev/null; then
    echo "ERROR: gaia_catalog import check failed (needed for generate_from_gaia)."
    echo "       $VENV/bin/python3 -c 'import gaia_catalog'"
    exit 1
fi

echo "  All Python packages installed."

# ---------------------------------------------------------------------------
# 3. Clone eFinder_cli (tetra3rs_mp branch)
# ---------------------------------------------------------------------------
echo ""
echo "[4/9] Cloning eFinder_cli (tetra3rs_mp branch)..."
REPO_URL="https://github.com/mconsidine/eFinder_cli.git"
REPO_DIR="$EFINDER_HOME/eFinder_cli"
cd "$EFINDER_HOME"
if [ ! -d "$REPO_DIR" ]; then
    sudo -u "$EFINDER_USER" git clone --depth 1 --branch tetra3rs_mp "$REPO_URL" "$REPO_DIR"
else
    echo "  Repo already present — removing and recloning..."
    sudo rm -rf "$REPO_DIR"
    sudo -u "$EFINDER_USER" git clone --depth 1 --branch tetra3rs_mp "$REPO_URL" "$REPO_DIR"
fi

# ---------------------------------------------------------------------------
# 4. Directory structure and file deployment
# ---------------------------------------------------------------------------
echo ""
echo "[5/9] Setting up directory structure..."
mkdir -p "$EFINDER_HOME/Solver/images"
mkdir -p "$EFINDER_HOME/uploads"
sudo chmod a+rwx "$EFINDER_HOME/uploads"
sudo chmod a+rwx "$EFINDER_HOME/Solver/images"
sudo chmod a+rwx "$EFINDER_HOME"

find "$REPO_DIR/Solver" -maxdepth 1 -type f | while read -r f; do
    cp "$f" "$EFINDER_HOME/Solver/"
done
sudo chown -R "$EFINDER_USER:$EFINDER_USER" "$EFINDER_HOME/Solver"

# RAM-backed tmpfs mounts — reduces SD card wear from debug image saves
grep -q "/var/tmp" /etc/fstab || \
    echo "tmpfs /var/tmp tmpfs nodev,nosuid,size=100M 0 0" | \
    sudo tee -a /etc/fstab > /dev/null
grep -q "$EFINDER_HOME/Solver/images" /etc/fstab || \
    echo "tmpfs $EFINDER_HOME/Solver/images tmpfs nodev,nosuid,size=10M 0 0" | \
    sudo tee -a /etc/fstab > /dev/null
sudo mount -a || echo "WARNING: mount -a had errors"

# ---------------------------------------------------------------------------
# 5. Star database generation (tetra3rs native, bundled Gaia + Hipparcos)
# ---------------------------------------------------------------------------
echo ""
echo "[6/9] Generating tetra3rs star database..."
DB_MAX_FOV=11
DB_MAG=8
DB_CACHE="$EFINDER_HOME/Solver/t3rs_fov${DB_MAX_FOV}_mag${DB_MAG}.bin"
DB_FIXED="$EFINDER_HOME/Solver/efinder-tetra-database.bin"

if [ ! -f "$DB_CACHE" ]; then
    echo "  Building database (max_fov=$DB_MAX_FOV, star_max_magnitude=$DB_MAG)..."
    echo "  Uses bundled Gaia DR3 + Hipparcos catalog — no download needed."
    echo "  This will take several minutes on Pi Zero 2W..."
    GEN_SCRIPT="/tmp/gen_database_tetra3rs_mp.py"
    echo "import tetra3rs"                                                  > "$GEN_SCRIPT"
    echo "db = tetra3rs.SolverDatabase.generate_from_gaia("               >> "$GEN_SCRIPT"
    echo "    max_fov_deg=${DB_MAX_FOV},"                                  >> "$GEN_SCRIPT"
    echo "    star_max_magnitude=${DB_MAG}.0,"                             >> "$GEN_SCRIPT"
    echo "    patterns_per_lattice_field=50,"                              >> "$GEN_SCRIPT"
    echo "    epoch_proper_motion_year=2026,"                              >> "$GEN_SCRIPT"
    echo "    verification_stars_per_fov=100,"                             >> "$GEN_SCRIPT"
    echo ")"                                                               >> "$GEN_SCRIPT"
    echo "db.save_to_file('${DB_CACHE}')"                                  >> "$GEN_SCRIPT"
    echo "print('Database saved: stars=%d patterns=%d' % (db.num_stars, db.num_patterns))" >> "$GEN_SCRIPT"
    sudo -u "$EFINDER_USER" "$VENV/bin/python3" "$GEN_SCRIPT"
    rm -f "$GEN_SCRIPT"
    echo "  Database generation complete."
else
    echo "  Cache $DB_CACHE already exists — skipping generation."
fi

echo "  Copying to fixed load name: efinder-tetra-database.bin"
sudo -u "$EFINDER_USER" cp "$DB_CACHE" "$DB_FIXED"
echo "  Database ready."

# ---------------------------------------------------------------------------
# 6. Samba share
# ---------------------------------------------------------------------------
echo ""
echo "[7/9] Configuring Samba file share..."
if ! grep -q "\[efindershare\]" /etc/samba/smb.conf; then
    echo ""                              | sudo tee -a /etc/samba/smb.conf > /dev/null
    echo "[efindershare]"               | sudo tee -a /etc/samba/smb.conf > /dev/null
    echo "path = /home/efinder"         | sudo tee -a /etc/samba/smb.conf > /dev/null
    echo "writeable = Yes"              | sudo tee -a /etc/samba/smb.conf > /dev/null
    echo "create mask = 0777"           | sudo tee -a /etc/samba/smb.conf > /dev/null
    echo "directory mask = 0777"        | sudo tee -a /etc/samba/smb.conf > /dev/null
    echo "public = no"                  | sudo tee -a /etc/samba/smb.conf > /dev/null
fi
(echo "$SAMBA_PASS"; echo "$SAMBA_PASS") | sudo smbpasswd -s -a "$EFINDER_USER"
sudo systemctl enable smbd nmbd
sudo systemctl restart smbd nmbd
echo "  Samba configured."

# ---------------------------------------------------------------------------
# 7. Apache / web interface
# ---------------------------------------------------------------------------
echo ""
echo "[8/9] Configuring Apache/PHP web server..."
sudo mkdir -p /var/www/html
sudo cp -r "$REPO_DIR/Solver/www/"* /var/www/html/ 2>/dev/null || true
[ -f /var/www/html/index.html ] && \
    sudo mv /var/www/html/index.html /var/www/html/apacheindex.html
echo "Timeout 3600" | sudo tee /etc/apache2/conf-available/efinder.conf > /dev/null
sudo a2enconf efinder
sudo chmod -R 755 /var/www/html
sudo chown -R www-data:www-data /var/www/html
sudo systemctl enable apache2
sudo systemctl restart apache2

# ---------------------------------------------------------------------------
# 8. Boot firmware (config.txt and cmdline.txt)
# ---------------------------------------------------------------------------
echo ""
echo "[9/9] Writing /boot/firmware/config.txt..."
BOOT_CONFIG=/boot/firmware/config.txt
[ ! -f "${BOOT_CONFIG}.bak" ] && sudo cp "$BOOT_CONFIG" "${BOOT_CONFIG}.bak"

sudo sed -i \
    -e '/^\s*dtoverlay=vc4-kms-v3d/s/^/#/' \
    -e '/^\s*max_framebuffers=/s/^/#/' \
    "$BOOT_CONFIG"

sudo sed -i \
    -e '/^\s*dtoverlay=dwc2/d' \
    -e '/^\s*enable_uart/d' \
    -e '/^\s*dtoverlay=arducam/d' \
    -e '/^\s*dtoverlay=imx477/d' \
    -e '/^\s*camera_auto_detect/d' \
    -e '/^# --- eFinder additions/d' \
    -e '/^# IMX477 camera/d' \
    -e '/^# USB gadget/d' \
    "$BOOT_CONFIG"

echo ""                                                          | sudo tee -a "$BOOT_CONFIG" > /dev/null
echo "# --- eFinder additions ---"                               | sudo tee -a "$BOOT_CONFIG" > /dev/null
echo "# IMX477 camera (RPi HQ Camera or Arducam IMX477)"         | sudo tee -a "$BOOT_CONFIG" > /dev/null
echo "camera_auto_detect=0"                                      | sudo tee -a "$BOOT_CONFIG" > /dev/null
echo "dtoverlay=imx477"                                          | sudo tee -a "$BOOT_CONFIG" > /dev/null
echo ""                                                          | sudo tee -a "$BOOT_CONFIG" > /dev/null
echo "# USB gadget serial (/dev/ttyUSB0 on host, /dev/ttyGS0 on Pi)" | sudo tee -a "$BOOT_CONFIG" > /dev/null
echo "dtoverlay=dwc2,dr_mode=peripheral"                         | sudo tee -a "$BOOT_CONFIG" > /dev/null
echo "enable_uart=1"                                             | sudo tee -a "$BOOT_CONFIG" > /dev/null

CMDLINE=/boot/firmware/cmdline.txt
sudo sed -i 's/ modules-load=dwc2,g_serial//g' "$CMDLINE"
sudo sed -i 's/ modules-load=dwc2,g_cdc//g'    "$CMDLINE"
if ! grep -q "modules-load=dwc2,g_serial" "$CMDLINE"; then
    sudo sed -i 's/rootwait/rootwait modules-load=dwc2,g_serial/' "$CMDLINE"
fi

# ---------------------------------------------------------------------------
# WiFi AP
# ---------------------------------------------------------------------------
echo ""
echo "[AP] Configuring NetworkManager WiFi hotspot..."
sudo rfkill unblock wifi
for f in /var/lib/systemd/rfkill/*:wlan; do
    [ -f "$f" ] && echo 0 | sudo tee "$f" > /dev/null
done

# WiFi regulatory country. Defaults to US. For other countries, set the
# WIFI_COUNTRY environment variable before running the installer, e.g.:
#     WIFI_COUNTRY=GB sudo -E bash install.sh
# (The -E preserves the environment across sudo.) CI-built images already
# have this baked in by the workflow; this call just confirms it.
WIFI_COUNTRY="${WIFI_COUNTRY:-US}"
echo "  WiFi country: $WIFI_COUNTRY"
sudo raspi-config nonint do_wifi_country "$WIFI_COUNTRY" 2>/dev/null || true

MAC=$(cat /sys/class/net/wlan0/address 2>/dev/null || \
      ip link show wlan0 | awk '/ether/{print $2}')
LAST4=$(echo "$MAC" | tr -d ':' | tail -c 5)
SSID="efinder${LAST4}"

echo "ssid:${SSID}"          > "$EFINDER_HOME/Solver/default_hotspot.txt"
echo "password:${WIFI_PASS}" >> "$EFINDER_HOME/Solver/default_hotspot.txt"
sudo chown "$EFINDER_USER:$EFINDER_USER" "$EFINDER_HOME/Solver/default_hotspot.txt"

sudo nmcli connection delete "efinder-ap" 2>/dev/null || true
sudo nmcli connection add \
    type wifi ifname wlan0 con-name "efinder-ap" \
    autoconnect yes ssid "$SSID" -- \
    wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$WIFI_PASS" \
    wifi.mode ap wifi.band bg wifi.channel 6 \
    ipv4.method shared ipv4.addresses "192.168.50.1/24" \
    ipv6.method disabled connection.autoconnect-priority 100
sudo nmcli connection delete "preconfigured" 2>/dev/null || true
echo "  AP profile created (SSID: $SSID)"

# ---------------------------------------------------------------------------
# Helper scripts (ap.sh, station.sh)
# ---------------------------------------------------------------------------
echo "[AP] Writing helper scripts..."

AP="$EFINDER_HOME/ap.sh"
cat > "$AP" <<'EOF'
#!/bin/bash
# ap.sh — switch to Access Point (hotspot) mode
set -e
echo "Switching to AP mode..."
ACTIVE=$(nmcli -t -f NAME,TYPE con show --active \
    | grep wifi | grep -v efinder-ap | cut -d: -f1)
nmcli connection down "$ACTIVE" 2>/dev/null || true
nmcli connection up efinder-ap
echo "AP active. SSH: ssh efinder@192.168.50.1"
EOF
sudo chmod +x "$AP"
sudo chown "$EFINDER_USER:$EFINDER_USER" "$AP"

ST="$EFINDER_HOME/station.sh"
cat > "$ST" <<'EOF'
#!/bin/bash
# station.sh — connect to external WiFi (station mode)
# Usage: ~/station.sh [ssid] [password]
#
# Builds an explicit NetworkManager profile rather than relying on
# `nmcli device wifi connect`'s security-mode inference, which fails
# with "key-mgmt: property is missing" when the scan cache is stale
# or incomplete.
set -e

_scan() {
    # Trigger a rescan and wait up to 8 s for it to complete. The Pi Zero 2W
    # has a single radio; if it just came off AP mode the scan takes a
    # moment to populate.
    nmcli device wifi rescan 2>/dev/null || true
    for i in $(seq 1 8); do
        if nmcli -f IN-USE,SSID,SECURITY device wifi list --rescan no \
             2>/dev/null | grep -q '[A-Za-z0-9]'; then
            return 0
        fi
        sleep 1
    done
}

if [ -z "$1" ]; then
    _scan
    echo "Available networks:"
    nmcli -f SSID,SIGNAL,SECURITY device wifi list --rescan no | head -20
    read -rp "Enter SSID: " SSID
    read -rsp "Enter password (blank=open): " PASSWORD; echo ""
else
    SSID="$1"; PASSWORD="${2:-}"
    _scan
fi
[ -z "$SSID" ] && { echo "ERROR: SSID empty"; exit 1; }

# Look up the security type for this SSID from the scan. If the network
# is not visible, fail clearly rather than building a bad profile.
SEC=$(nmcli -t -f SSID,SECURITY device wifi list --rescan no \
      | awk -F: -v s="$SSID" '$1==s {print $2; exit}')

if [ -z "$SEC" ]; then
    echo "ERROR: SSID '$SSID' not found in scan."
    echo "Visible networks:"
    nmcli -f SSID,SIGNAL,SECURITY device wifi list --rescan no | head -20
    exit 1
fi

echo "Connecting to: $SSID  (security: $SEC)"

# Remove any stale profile with the same name so we rebuild from scratch.
nmcli connection delete "$SSID" 2>/dev/null || true

# Drop AP so the radio is free to associate.
nmcli connection down efinder-ap 2>/dev/null || true

# Build an explicit profile. Handle the three common cases:
#   open       -> no security
#   WPA/WPA2   -> wpa-psk
#   WPA3 (SAE) -> sae
case "$SEC" in
    *SAE*|*WPA3*)
        KEY_MGMT="sae"
        ;;
    *WPA*|*PSK*)
        KEY_MGMT="wpa-psk"
        ;;
    ""|--)
        KEY_MGMT=""
        ;;
    *)
        echo "WARNING: unrecognized security '$SEC', trying wpa-psk"
        KEY_MGMT="wpa-psk"
        ;;
esac

if [ -n "$KEY_MGMT" ]; then
    if [ -z "$PASSWORD" ]; then
        echo "ERROR: network '$SSID' requires a password ($SEC)"
        nmcli connection up efinder-ap 2>/dev/null || true
        exit 1
    fi
    # psk-flags=0 tells NM to store the PSK in the connection profile file
    # itself, not rely on a secret agent (gnome-keyring / polkit / etc.).
    # Pi OS Lite has no such agent running, so the default flag (1 =
    # agent-owned) causes `connection up` to fail with "Secrets were
    # required, but not provided" even though we just passed the PSK to
    # `connection add`. Flag 0 sidesteps that entire dance.
    nmcli connection add type wifi ifname wlan0 con-name "$SSID" \
        ssid "$SSID" \
        wifi-sec.key-mgmt "$KEY_MGMT" \
        wifi-sec.psk "$PASSWORD" \
        wifi-sec.psk-flags 0 \
        connection.autoconnect no \
        >/dev/null
else
    nmcli connection add type wifi ifname wlan0 con-name "$SSID" \
        ssid "$SSID" \
        connection.autoconnect no \
        >/dev/null
fi

# Bring the profile up.
if ! nmcli connection up "$SSID"; then
    echo "Failed to activate '$SSID' — returning to AP mode"
    nmcli connection delete "$SSID" 2>/dev/null || true
    nmcli connection up efinder-ap 2>/dev/null || true
    exit 1
fi

# Wait for a non-AP IP address.
IP=""
for i in $(seq 1 20); do
    IP=$(ip -4 addr show wlan0 \
         | grep -oP "(?<=inet )[\d.]+" | grep -v "192\.168\.50\." | head -1)
    [ -n "$IP" ] && break
    sleep 1
done
[ -z "$IP" ] && IP=$(hostname -I | awk '{print $1}')

echo "Connected to: $SSID  IP: $IP"
echo "SSH: ssh efinder@$IP"
echo "Run ~/ap.sh to return to AP mode."
EOF
sudo chmod +x "$ST"
sudo chown "$EFINDER_USER:$EFINDER_USER" "$ST"

# ---------------------------------------------------------------------------
# polkit rule — efinder can run nmcli without sudo
# ---------------------------------------------------------------------------
sudo mkdir -p /etc/polkit-1/rules.d
POLKIT=/etc/polkit-1/rules.d/50-efinder-nm.rules
cat <<'EOF' | sudo tee "$POLKIT" > /dev/null
polkit.addRule(function(action, subject) {
    if (action.id.indexOf("org.freedesktop.NetworkManager.") == 0 &&
        subject.user == "efinder") {
        return polkit.Result.YES;
    }
});
EOF
sudo chmod 644 "$POLKIT"

# ---------------------------------------------------------------------------
# USB serial tether getty
# ---------------------------------------------------------------------------
echo "[USB] Enabling getty on USB serial gadget (ttyGS0)..."
sudo mkdir -p /etc/systemd/system/serial-getty@ttyGS0.service.d
cat <<'EOF' | sudo tee /etc/systemd/system/serial-getty@ttyGS0.service.d/override.conf > /dev/null
[Service]
ExecStart=
ExecStart=-/sbin/agetty -L -i 115200 ttyGS0
EOF
sudo systemctl enable serial-getty@ttyGS0.service

# ---------------------------------------------------------------------------
# Interface / peripheral setup — BEFORE SSH restart (raspi-config footgun)
# ---------------------------------------------------------------------------
sudo raspi-config nonint do_i2c 0 2>/dev/null || true
sudo raspi-config nonint do_serial_cons 1 2>/dev/null || true

grep -q "vm.swappiness" /etc/sysctl.conf || \
    echo 'vm.swappiness = 0' | sudo tee -a /etc/sysctl.conf > /dev/null

# ---------------------------------------------------------------------------
# SSH — done AFTER raspi-config calls so PasswordAuthentication is not reset
# ---------------------------------------------------------------------------
sudo systemctl enable --now ssh
sudo sed -i 's/^#*PasswordAuthentication.*/PasswordAuthentication yes/' \
    /etc/ssh/sshd_config
sudo systemctl restart ssh

# ---------------------------------------------------------------------------
# Sudoers
# ---------------------------------------------------------------------------
SUDOERS_FILE=/etc/sudoers.d/efinder
cat <<EOF | sudo tee "$SUDOERS_FILE" > /dev/null
$EFINDER_USER ALL=(ALL) NOPASSWD: /bin/date
$EFINDER_USER ALL=(ALL) NOPASSWD: /usr/bin/date
$EFINDER_USER ALL=(ALL) NOPASSWD: /bin/systemctl restart efinder
$EFINDER_USER ALL=(ALL) NOPASSWD: /usr/bin/systemctl restart efinder
EOF
sudo chmod 440 "$SUDOERS_FILE"

# ---------------------------------------------------------------------------
# OTA update helper (ported from tiny_img)
# Extracts /home/efinder/uploads/efinderUpdate.zip to / at boot, then reboots.
# ---------------------------------------------------------------------------
APPLY="$EFINDER_HOME/Solver/apply_update.sh"
cat <<'EOF' | sudo tee "$APPLY" > /dev/null
#!/bin/bash
ZIPFILE="/home/efinder/uploads/efinderUpdate.zip"
[ ! -f "$ZIPFILE" ] && echo "No update zip." && exit 0
echo "Applying update..."
if unzip -o "$ZIPFILE" -d /; then
    find / -name "*.py" -newer "$ZIPFILE" -exec chmod a+rwx {} \; 2>/dev/null || true
    rm -f "$ZIPFILE"
    echo "Update applied — rebooting."
    systemctl reboot
else
    echo "Update failed — removing zip."
    rm -f "$ZIPFILE"
fi
EOF
sudo chmod 755 "$APPLY"
sudo chown root:root "$APPLY"

# Install unzip so apply_update.sh works
sudo apt-get install -y unzip

# ---------------------------------------------------------------------------
# Systemd services
# ---------------------------------------------------------------------------
echo ""
echo "[SVC] Installing systemd services..."

CPU_SVC=/etc/systemd/system/cpu-performance.service
cat <<'EOF' | sudo tee "$CPU_SVC" > /dev/null
[Unit]
Description=Set CPU governor to performance mode
After=sysinit.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/bin/sh -c 'echo performance | tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor'
StandardOutput=journal

[Install]
WantedBy=multi-user.target
EOF

# Item 8: WiFi power save off.
# In AP mode on Pi Zero 2W, WiFi PSM can add 20-100ms latency jitter to
# LX200 TCP packets (SkySafari polls :GR/:GD many times per second).
# This keeps the radio awake whenever wlan0 is up.
sudo apt-get install -y --no-install-recommends iw wireless-tools
WPS_SVC=/etc/systemd/system/wifi-powersave-off.service
cat <<'EOF' | sudo tee "$WPS_SVC" > /dev/null
[Unit]
Description=Disable WiFi power save on wlan0 (eFinder latency optimization)
After=NetworkManager.service sys-subsystem-net-devices-wlan0.device
Wants=NetworkManager.service

[Service]
Type=oneshot
RemainAfterExit=yes
# Try iw first (Bookworm/Trixie default); fall back to iwconfig if absent.
ExecStart=/bin/sh -c '/usr/sbin/iw dev wlan0 set power_save off 2>/dev/null || /usr/sbin/iwconfig wlan0 power off 2>/dev/null || true'
StandardOutput=journal

[Install]
WantedBy=multi-user.target
EOF

UPD_SVC=/etc/systemd/system/efinder-update.service
cat <<'EOF' | sudo tee "$UPD_SVC" > /dev/null
[Unit]
Description=eFinder OTA update checker
Before=efinder.service
After=local-fs.target

[Service]
Type=oneshot
RemainAfterExit=yes
User=root
WorkingDirectory=/home/efinder
ExecStart=/bin/bash /home/efinder/Solver/apply_update.sh
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

EF_SVC=/etc/systemd/system/efinder.service
cat <<EOF | sudo tee "$EF_SVC" > /dev/null
[Unit]
Description=eFinder tetra3rs_mp plate solver
After=local-fs.target network.target efinder-update.service

[Service]
Type=simple
User=efinder
WorkingDirectory=/home/efinder/Solver
Environment=PATH=/home/efinder/venv-efinder/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
ExecStartPre=/bin/sleep 2
ExecStart=/home/efinder/venv-efinder/bin/python /home/efinder/Solver/$APP_SCRIPT
Restart=on-failure
RestartSec=15
StartLimitIntervalSec=120
StartLimitBurst=4
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable cpu-performance.service
sudo systemctl enable wifi-powersave-off.service
sudo systemctl enable efinder-update.service
sudo systemctl enable efinder.service

sudo nmcli connection modify "preconfigured" autoconnect no 2>/dev/null || true
sudo rm -f /etc/cron.d/efinder
echo "  All services installed and enabled."

# ---------------------------------------------------------------------------
# Slim the image (ported from tiny_img — a bit more aggressive than tetra3rs')
# ---------------------------------------------------------------------------
echo ""
echo "[slim] Removing build-time packages and cleaning cache..."
sudo apt-get purge -y \
    protobuf-compiler \
    build-essential \
    cmake \
    pkg-config \
    libssl-dev \
    2>/dev/null || true
sudo apt-get autoremove -y
sudo apt-get clean
sudo rm -rf /usr/share/doc/* /usr/share/man/* /usr/share/info/*
echo "  Done."

# ---------------------------------------------------------------------------
# Done
# ---------------------------------------------------------------------------
echo ""
echo "============================================================================="
echo " Installation complete."
echo ""
echo "   WiFi AP  : SSID='$SSID'  Password='$WIFI_PASS'  IP=192.168.50.1"
echo "   SSH      : ssh efinder@192.168.50.1"
echo "   Tether   : screen /dev/ttyUSB0 115200"
echo "   SkySafari: connect to $SSID -> TCP port 4060 (LX200)"
echo "   Samba    : efindershare  user=efinder  pass=$SAMBA_PASS"
echo "   Logs     : journalctl -u efinder -f"
echo "   Helpers  : ~/ap.sh   ~/station.sh"
echo "============================================================================="

date > "$INSTALL_MARKER"

if [ "$NON_INTERACTIVE" = false ]; then
    read -rp "Reboot now? [y/N] " ans
    [[ "$ans" =~ ^[Yy]$ ]] && sudo reboot now
fi
