### About Tasks and Capabilities

<!--TODO: Add tutorial to create a new capability when capability plugins are supported-->
A capability represent tasks that a workcell can perform. When the workcell orchestrator receives a request to perform a task, it looks at the `type` field of the request to determine which behavior tree is to be used to complete the reqeust. To support an extensible framework for different tasks, capabilities define the behavior trees, actions and all other functions needed to perform a task and register the task types.

The steps to complete a task is defined by a behavior tree, which is represented by a xml document.

*Example of a minimal behavior tree*
```xml
<root main_tree_to_execute="MyTask">
  <BehaviorTree ID="MyTask">
    <Sequence name="root_sequence">
      <my_task.my_action_1 />
      <my_task.my_action_2 />
    </Sequence>
  </BehaviorTree>
</root>
```

Capabilities must provide the action nodes needed to perform the task they are registering, in the above example, `my_task.my_action_1` and `my_task.my_action_2` are action nodes that must be provided by the capability. Capabilities register actions by calling the registration functions in `Context::bt_factory`.

*Example of a behavior tree action*
```cpp
context->bt_factory.registerSimpleAction("my_task.my_action_1",
    [](BT::TreeNode& bt)
    {
      // Do stuff
      return BT::NodeStatus::SUCCESS;
    }, {});
```

> **Note: In order to avoid name clashes, it is highly recommended to prefix the action nodes with the capability id.**

> Note: Prefixes are used instead of xml namespaces because behaviortree.cpp does not support xml namespaces (as of v3.8)

It can then register the task itself

```cpp
  context->task_parser.register_handler("my_task",
    [](std::istream& yaml_stream)
    {
      return std::make_unique<MyTask>();
    });
```

With these, the workcell orchestrator can now handle tasks of type `my_task`.
