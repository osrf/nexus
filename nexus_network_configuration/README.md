# Network Config Generator

This package simplifies the Zenoh DDS Bridge setup for multiple NEXUS orchestrators through the generation of Zenoh bridge configurations from the following 2 files:
1. NEXUS Network Configuration: This YAML file describes the properties of different machines such as ROS Domain ID, Zenoh bridge TCP connection endpoints etc. Refer to [Quick Start](#quick-start) for an example
2. REDF Configuration: This YAML file describes the ROS Endpoints of the NEXUS orchestrators

# First-time setup for deploying a LOCALHOST only environment

1. Make sure that `rmw_zenoh_cpp` is installed and made the default middleware, and that multicast on loopback interface is enabled for localhost-only communication.
```
./scripts/set_up_network.sh
```

2. Create a NEXUS Network configuration. An example is provided in the `nexus_poc` package at [nexus_network_config.yaml](../nexus_poc/config/zenoh/nexus_network_config.yaml) Here we have 1 system orchestrator connected to 3 workcell orchestrators, all on different domain IDs from 14 to 17.
You can refer to [nexus_network_schema.json](schemas/nexus_network_schema.json) for the config schema.

```yaml
# Mode of Zenoh bridge, specified as either 'client' or 'peer'.
# Use 'client' when there is an available Zenoh router, otherwise
# use 'peer' for a distributed system.
mode: peer
# When true, discovery info is forwarded to the remote plugins/bridges
forward_discovery: false
# When true, activates a REST API used to administer Zenoh Bridge configurations
enable_rest_api: true

system_orchestrators:
  - ros_namespace: system_orchestrator # ROS Namespace of the endpoints
    # ROS Domain ID
    domain_id: 14
    # Listening TCP Address of Zenoh bridge
    tcp_listen: "localhost:7447"
    # HTTP Port for the REST API
    rest_api_http_port: 8000

workcell_orchestrators:
  - ros_namespace: workcell_1
    domain_id: 15
    tcp_connect: "localhost:7447"
    rest_api_http_port: 8001

  - ros_namespace: workcell_2
    domain_id: 16
    tcp_connect: "localhost:7447"
    rest_api_http_port: 8002

  - ros_namespace: workcell_3
    domain_id: 17
    tcp_connect: "localhost:7447"
    rest_api_http_port: 8003
```

3. Generate zenoh bridge configurations from NEXUS Network config and REDF config. The `ZENOH_CONFIGS_OUTPUT_DIRECTORY` will be where your zenoh bridge configurations will be saved.
```bash
ros2 run nexus_network_configuration nexus_network_configuration -n <PATH_TO_NEXUS_NETWORK_CONFIG> -r <PATH_TO_REDF_CONFIGS> -o <ZENOH_CONFIGS_OUTPUT_DIRECTORY>
```
