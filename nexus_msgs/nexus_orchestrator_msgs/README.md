# nexus_orchestrator_msgs

## Description
This is a package for ROS 2 interfaces between system orchestrators and workcells

### Messages
  * [WorkcellDescription.msg](msg/WorkcellDescription.msg) : Message for providing workcell description when registering a workcell
  * [WorkcellState.msg](msg/WorkcellState.msg) : Message for feedback on workcell state during execution of a task
### Services
  * [RegisterWorkcell.srv](srv/RegisterWorkcell.srv) : Service for registering a new workcell with the system orchestrator
### Actions
  * [WorkcellTask.action](action/WorkcellTask.action) : Action to send task request to a workcell
