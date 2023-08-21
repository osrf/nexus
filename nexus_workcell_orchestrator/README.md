### nexus_workcell_orchestrator

The workcell orchestrator:
1. ~~discover it's capabilities~~ wip
1. report it's ability to perform a task
1. performs a task request
1. register itself to the system orchestrator
1. monitor the state of controllers
1. request controllers to change states

### Supported Capabilities

The capabilities supported by a workcell must be provided by the `capabilities` ROS parameter. The parameter must be an array of strings listing all the ids of the capabilities supported. Also depending on the capabilities provided, other parameters must also be provided.

### Remapping Task Types

The task types received by a specific workcell can be remapped to a capability via `remap_task_types` ROS Parameter. This parameter is in the form of a YAML string, where the key is the capability to be mapped to and the value is an array specifying what task types to map from. One such example is provided:
```yaml
{
    pick_and_place: [pick_and_place_productA, pick_and_place_productB],
}
```
In the example above, when the orchestrator receives a work order with steps containing either `pick_and_place_productA` or `pick_and_place_productB`, it will consider it to be the same as `pick_and_place` and will execute the same `pick_and_place.xml` behavior tree for either.
