# nexus_transporter_msgs
A package containing ROS 2 interfaces for communications with systems providing
transportation services.

* [TransportationRequest.msg](msg/TransportationRequest.msg) : A message for describing the request
* [TransporterState.msg](msg/TransporterState.msg): A message describing the state of a transporter
* [IsTransporterAvailable.srv](srv/IsTransporterAvailable.srv) : A service for checking if a capable transporter is available to fulfil the described request
* [Transport.action](action/Transport.action): An action for fulfilling a transportation request
