import sys
from uuid import uuid4

import rclpy
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import State, Transition

if len(sys.argv) != 2:
    print("Usage: python3 activate_node.py <target_node>")
    exit(1)

rclpy.init()
suffix = str(uuid4()).replace("-", "_")
node = rclpy.create_node(f"lifecycle_activator_{suffix}")
target = sys.argv[1]

service_name = f"{target}/get_state"
get_state = node.create_client(GetState, service_name)
if not get_state.wait_for_service(10):
    node.get_logger().error(f"Unable to find service [{service_name}]")
    exit(1)
fut = get_state.call_async(GetState.Request())
rclpy.spin_until_future_complete(node, fut)
resp: GetState.Response = fut.result()
transitions = [Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE]
if resp.current_state.id == State.PRIMARY_STATE_UNCONFIGURED:
    current_transition_idx = 0
elif resp.current_state.id == State.PRIMARY_STATE_INACTIVE:
    current_transition_idx = 1
else:
    node.get_logger().error(
        f"Unable to transition from current state [{resp.current_state}]"
    )
    exit(1)

service_name = f"{target}/change_state"
change_state = node.create_client(ChangeState, service_name)
if not get_state.wait_for_service(10):
    node.get_logger().error(f"Unable to find service [{service_name}]")
    exit(1)
while current_transition_idx < len(transitions):
    current_transition = transitions[current_transition_idx]
    fut = change_state.call_async(
        ChangeState.Request(transition=Transition(id=current_transition))
    )
    rclpy.spin_until_future_complete(node, fut)
    resp: ChangeState.Response = fut.result()
    if not resp.success:
        node.get_logger().error(
            f"Failed to transition {target} to [{current_transition}]"
        )
    current_transition_idx += 1

node.get_logger().info(f"Successfully transitioned {target} to active")
