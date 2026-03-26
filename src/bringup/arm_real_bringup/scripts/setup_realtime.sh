#!/bin/bash
# Setup realtime scheduling permissions for EtherCAT control.
# Requires sudo. Run once per machine, then log out and back in.
#
# Without RT permissions, the ros2_control_node cannot use SCHED_FIFO,
# causing control loop overruns that make EtherCAT slaves drop from OP state.

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo "=== EtherCAT Realtime Scheduling Setup ==="
echo ""

# Check if running as root or with sudo
if [ "$EUID" -ne 0 ]; then
    echo -e "${RED}Error: This script must be run with sudo${NC}"
    echo "  sudo bash $0"
    exit 1
fi

ACTUAL_USER="${SUDO_USER:-$USER}"

# Step 1: Create realtime group
echo -n "Creating 'realtime' group... "
if groupadd -f realtime 2>/dev/null; then
    echo -e "${GREEN}OK${NC}"
else
    echo -e "${YELLOW}already exists${NC}"
fi

# Step 2: Add user to realtime group
echo -n "Adding '$ACTUAL_USER' to 'realtime' group... "
if usermod -aG realtime "$ACTUAL_USER"; then
    echo -e "${GREEN}OK${NC}"
else
    echo -e "${RED}FAILED${NC}"
    exit 1
fi

# Step 3: Create limits.d config
LIMITS_FILE="/etc/security/limits.d/99-ros2-realtime.conf"
echo -n "Writing $LIMITS_FILE... "
cat > "$LIMITS_FILE" << 'EOF'
# Realtime scheduling for ros2_control EtherCAT
# Allows members of the 'realtime' group to:
#   - Use SCHED_FIFO/SCHED_RR with priority up to 99
#   - Lock memory (mlockall) for deterministic latency
#   - Set process nice values down to -20
@realtime   -  rtprio     99
@realtime   -  memlock    unlimited
@realtime   -  nice       -20
EOF
echo -e "${GREEN}OK${NC}"

# Step 4: Verify
echo ""
echo "=== Verification ==="
echo -n "  limits.d file exists: "
[ -f "$LIMITS_FILE" ] && echo -e "${GREEN}YES${NC}" || echo -e "${RED}NO${NC}"
echo -n "  '$ACTUAL_USER' in 'realtime' group: "
if id -nG "$ACTUAL_USER" | grep -qw realtime; then
    echo -e "${GREEN}YES${NC}"
else
    echo -e "${YELLOW}pending (need re-login)${NC}"
fi

echo ""
echo -e "${GREEN}Done!${NC} You must ${YELLOW}log out and back in${NC} for group changes to take effect."
echo ""
echo "After re-login, verify with:"
echo "  ulimit -r    # should show 99"
echo "  id -nG       # should include 'realtime'"
