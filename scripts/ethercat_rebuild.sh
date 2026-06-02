#!/usr/bin/env bash
#
# Rebuild & reinstall the IgH EtherCAT master.
# Portable: the repo is wherever this script lives (works on any PC/user,
# and through symlinks). Secure Boot is disabled -> no module signing.
#
set -euo pipefail

# Resolve the repo as the directory containing this script (follows symlinks).
REPO="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
KVER="$(uname -r)"
HDR="/usr/src/linux-headers-$KVER"
CONF="/usr/local/etc/ethercat.conf"

echo ">> EtherCAT rebuild for kernel $KVER"
echo ">> Repo: $REPO"

[ -f "$REPO/configure" ] || { echo "!! Not an EtherCAT repo (no ./configure): $REPO"; exit 1; }
[ -d "$HDR" ]            || { echo "!! Kernel headers missing: $HDR"; echo "   sudo apt install linux-headers-$KVER"; exit 1; }
cd "$REPO"

echo ">> Stopping service and unloading old modules"
sudo systemctl stop ethercat 2>/dev/null || true
sudo modprobe -r ec_generic ec_master 2>/dev/null || true

echo ">> Backing up ethercat.conf"
sudo cp "$CONF" "$CONF.bak" 2>/dev/null || true

echo ">> Clean + configure + build"
make clean >/dev/null
./configure --with-linux-dir="$HDR" --enable-generic --prefix=/usr/local >/dev/null
make -j"$(nproc)"
make modules

echo ">> Installing"
sudo make modules_install
sudo make install
sudo depmod

echo ">> Restoring ethercat.conf"
[ -f "$CONF.bak" ] && sudo cp "$CONF.bak" "$CONF"
# Force DEVICE_MODULES lowercase -- ethercatctl does modprobe ec_${MODULE}
sudo sed -i 's/^DEVICE_MODULES=.*/DEVICE_MODULES="generic"/' "$CONF"

echo ">> Reloading systemd and starting service"
sudo systemctl daemon-reload
sudo systemctl start ethercat
sudo systemctl status ethercat --no-pager || true

echo ">> Loaded modules:"
lsmod | grep -E '^ec_' || echo "   (none -- check 'journalctl -u ethercat')"

echo ">> Done."
ethercat master || true
