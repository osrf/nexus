# Name of the station, must be unique within a workcell. This name may
# correspond to a waypoint in a navigation graph (e.g. for RMF).
string name

# Type of station
uint8 IO_TYPE_INPUT=0
uint8 IO_TYPE_OUTPUT=1
uint8 IO_TYPE_BIDIRECTIONAL=2
uint8 io_type
