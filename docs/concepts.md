# NEXUS Architecture and Concepts

## Guiding Principles

- **Hardware agnosticism**: The core logic of the system should be independent of the brand or type of hardware component.
- **Scalability**: The system should adapt to additions, subtractions or changes in layout of subsystems providing services.
- **Reusability**: To facilitate scalability, components of the system throughout the architecture stack should be designed for reuse
- **Reliability**: Ensure resilience to hard failures as the system scales
- **Speed**: Ensure efficient operations and minimize added latency

NEXUS is responsible for the coordination of actions within a facility composed of multiple heterogeneous workcells. We define a workcell as a subsystem that provides a specific capability such as pick and place of boxes. Such a workcell may in turn have several subsystems, eg, vision system for object detection and robotic arms for pick and place, etc. The fulfillment of a task would thus require coordination in execution of various subsystems within a workcell and potentially among workcells. A single node to manage such complex workflows is susceptible to a number of problems including

- **Robustness**: A single point of failure for the entire system
- **Scalability**: Adding new workcells to the system would require rewriting core coordination logic
- **Complexity**: Several asynchronous events need to be managed for each mechanical system
- **Network traffic**: The single node needs to discover every other node in the system and this would lead to a network storm of message and discovery packets. This would increase latency or worse lead to messages being dropped.

## Core Design Elements

![](media/nexus_architecture.png)

The adopted architecture is a decentralized yet modular with the following core concepts:

TODO
