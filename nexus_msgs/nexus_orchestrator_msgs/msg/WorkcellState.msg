# Unique identifier for workcell
string workcell_id

# Current work order being fulfilled
string work_order_id

# Current task being performed
string task_id

# [OPTIONAL] message for debugging
string message

uint8 status
uint8 STATUS_IDLE = 1
uint8 STATUS_BUSY = 2
