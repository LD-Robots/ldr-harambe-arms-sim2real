*General formulas*

Both TxPDO and RxPDO use the same formula:


output = factor × input + offset
Where:

TxPDO (state read)	RxPDO (command write)
input	raw PDO value	ros2_control interface value
output	ros2_control interface value	raw PDO value
factor	converts raw → engineering units	converts engineering units → raw
offset	in engineering units	in raw units
Factor relationship

cmd_factor = 1 / state_factor
Offset relationship
Given a known raw zero-point raw_zero (the raw encoder value when the joint is at 0 in your coordinate frame):

RxPDO (command):


offset_cmd = raw_zero
Because when command = 0: raw = factor × 0 + offset = raw_zero

TxPDO (state):


offset_state = -state_factor × raw_zero
Because when raw = raw_zero: state = state_factor × raw_zero + offset = 0

Or equivalently:


offset_state = -offset_cmd / cmd_factor
Verification
Both should produce 0 at the zero point:

State read: state_factor × raw_zero + offset_state = state_factor × raw_zero - state_factor × raw_zero = 0
Command write: cmd_factor × 0 + offset_cmd = raw_zero