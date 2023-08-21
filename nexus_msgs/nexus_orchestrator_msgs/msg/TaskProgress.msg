# [OPTIONAL] Time estimation of how long workcell will take to
# complete it's current task
builtin_interfaces/Time estimated_finish_time

# These 2 fields describe the progress of the task.
# max_progress describes the number of "units of work" of the task, current_progress is the
# number of units done. A max_progress of 0 means that progress is unknown.
# For example, a pick and place task that needs to place 2 items each on 3 items could be defined to
# have 6 units of work.
uint64 current_progress
uint64 max_progress
