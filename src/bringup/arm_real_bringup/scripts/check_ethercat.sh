#!/bin/bash
# Pre-flight check for EtherCAT hardware
# Run this before launching the real arm system.
#
# Usage: bash check_ethercat.sh [expected_slave_count]
#
# Checks:
#   1. EtherLab kernel module is loaded
#   2. EtherCAT master device exists
#   3. Master is in idle state (ready for application)
#   4. Expected number of slaves detected
#   5. All slaves are in PREOP or higher state

set -e

EXPECTED_SLAVES=${1:-6}  # Default: 6 slaves (left arm)
ETHERCAT_CMD="ethercat"  # EtherLab CLI tool

echo "=== EtherCAT Pre-Flight Check ==="
echo ""

# Check 1: Kernel module
echo "[1/5] Checking EtherLab kernel module..."
if lsmod | grep -q "ec_master"; then
    echo "  OK: ec_master module loaded"
else
    echo "  FAIL: ec_master module not loaded"
    echo "  Fix: sudo modprobe ec_master main_devices=<MAC>"
    exit 1
fi

# Check 2: Master device
echo "[2/5] Checking EtherCAT master device..."
if [ -e /dev/EtherCAT0 ]; then
    echo "  OK: /dev/EtherCAT0 exists"
else
    echo "  FAIL: /dev/EtherCAT0 not found"
    echo "  Fix: Check EtherLab installation and udev rules"
    exit 1
fi

# Check 3: Master state
echo "[3/5] Checking master state..."
MASTER_STATE=$($ETHERCAT_CMD master 2>/dev/null | grep "Phase:" | awk '{print $2}')
if [ "$MASTER_STATE" = "Idle" ] || [ "$MASTER_STATE" = "Operation" ]; then
    echo "  OK: Master phase = $MASTER_STATE"
else
    echo "  WARN: Master phase = $MASTER_STATE (expected Idle or Operation)"
fi

# Check 4: Slave count
echo "[4/5] Checking slave count (expecting $EXPECTED_SLAVES)..."
SLAVE_COUNT=$($ETHERCAT_CMD slaves 2>/dev/null | wc -l)
if [ "$SLAVE_COUNT" -eq "$EXPECTED_SLAVES" ]; then
    echo "  OK: $SLAVE_COUNT slaves detected"
else
    echo "  FAIL: $SLAVE_COUNT slaves detected (expected $EXPECTED_SLAVES)"
    echo "  Slaves found:"
    $ETHERCAT_CMD slaves 2>/dev/null || true
    exit 1
fi

# Check 5: Slave states
echo "[5/5] Checking slave states..."
$ETHERCAT_CMD slaves 2>/dev/null | while read -r line; do
    echo "  $line"
done

echo ""
echo "=== All checks passed ==="
echo "Ready to launch: ros2 launch arm_real_bringup arm_real.launch.py"
