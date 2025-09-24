# The workcell executing the task
string workcell_id

# Id of the task
string task_id

TaskProgress progress

# [OPTIONAL] messages for debugging
string[] messages

string[] error_messages

uint8 status
uint8 STATUS_NONE = 0
# Task is queued for execution
uint8 STATUS_QUEUED = 1
# Task is executing
uint8 STATUS_RUNNING = 2
# Task finished successfully
uint8 STATUS_FINISHED = 3
# Task is assigned to a workcell but not yet queued for execution.
uint8 STATUS_ASSIGNED = 4
# Task has failed
uint8 STATUS_FAILED = 5
