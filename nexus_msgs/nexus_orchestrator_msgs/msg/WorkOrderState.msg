# Id of the work order
string id

uint8 state
uint8 STATE_NONE = 0
uint8 STATE_BIDDING = 1
uint8 STATE_EXECUTING = 2
uint8 STATE_SUCCESS = 3
uint8 STATE_CANCELLED = 4
uint8 STATE_FAILED = 5

TaskState[] task_states
