# General setup (so far)

```bash
ros2 launch nexus_demos launch.py headless:=false run_workcell_1:=false run_workcell_2:=false use_zenoh_bridge:=false
```

```bash
ros2 run zenoh_security_tools generate_configs \
  --policy policy_main.xml \
  --router-config ~/nexus_workspaces/sros_demo/rmw_zenoh/rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5 \
  --session-config ~/nexus_workspaces/sros_demo/rmw_zenoh/rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5 \
  --ros-domain-id 0
```

This still generates a lot of weirdness, we will use the manual ACL method instead.
