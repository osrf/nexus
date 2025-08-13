# This message describes the workcell's endpoint namespace, physical information
# as well as it's capabilities

# Unique identifier for workcell, used in namespacing topic
# subscriptions, services and actions within the system orchestrator
string workcell_id

# User friendly name of the workcell
string workcell_name

# Level of the factory that the workcell is located on
string level_name

# Physical Location of the workcell with respect to a common reference point
geometry_msgs/Pose location

# Capabilities of the workcell, see README for nexus_workcell_orchestrator for list of capabilities supported
string[] capabilities

# A list of I/O stations for the workcell
WorkcellStation[] io_stations
