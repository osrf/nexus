## nexus_transporter

This package provides a pure virtual interface `Transporter` to be implemented to integrate a transportation system into Nexus.

Libraries that implement the `Transporter` class can be loaded through `pluginlib` at runtime by the `TransporterNode`.
Two reference implementations of `Transporter` are provided.

### Mock transporter

The mock transporter simulates a simple conveyor belt and has the following parameters:

* `x_increment` The distance in meters between stations.
* `destinations` An ordered list of named stations.
* `speed` Speed in m/s of the conveyor belt.

The plugin will then simulate a conveyor with a list of `destinations`, equally spaced by `x_increment`, moving at a fixed `speed`.

### RMF transporter

The RMF transporter provides an interface to a running RMF instance. The plugin will create a composed task with a series of pickup / dropoffs based on the requested itinerary and assess its feasibility by checking,
* that the requested destinations are present in the navigation graphs of the Open-RMF building map, only then will it
* start an Open-RMF dry-run bidding process with the created task, to determine which AMR should be assigned to the task, without starting the task

Once the itinerary has been accepted and the transporter starts the transportation task, the composed task will be directly dispatched to the assigned AMR.

The plugin implements the `Transporter::handle_signal` interface to signal AMRs when they can resume their itinerary, by publishing the corresponding dispensing or ingesting results.

Specifically, when an AMR reaches its last destination and it is marked as a `pickup`, the transportation request will be marked as complete and Open-RMF will begin publishing an `DispenserRequest` and wait for a matching `DispenserResult`.
As soon as the transporter's `handle_signal` function receives the `pickup` signal, the matching `DispenserResult` message will be published and the AMR will be released and complete its task.

#### Future work

* Move from hardcoded `pickup` final phase to a `dropoff `.
* Implement workcells with multiple input / output stations and use them for transportation requests.
