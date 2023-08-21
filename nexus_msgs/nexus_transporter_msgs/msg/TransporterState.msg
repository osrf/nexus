# The name of the transporter
string transporter

# The model name of the transporter
string model

# The optional id of the task being fulfilled by this transporter
string task_id

# The current location of the transporter
geometry_msgs/PoseStamped location

# The current state of the transporter
int32 state
uint32 STATE_UNAVAILABLE=0
uint32 STATE_IDLE=1
uint32 STATE_MOVING=2

# An optional estimate of how long it will take the transporter to fulfil its
# current task
builtin_interfaces/Duration estimated_finish_time
