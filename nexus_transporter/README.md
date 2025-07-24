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

The RMF transporter provides an interface to a running RMF instance. The plugin will create a composed task with a series of pickup / dropoffs based on the requested itinerary and assess its feasibility by checking that the requested destinations are present in the building map published by Open-RMF.
The time estimate is temporarily hardcoded to a fixed value.
and estimate `Itinerary` duration by submitting a task with the dry run feature, which allow it to request to the Open-RMF task planner what's the estimate for a specific task without actually starting it.
The node also uses the `Transporter::handle_signal` interface to signal AMRs when they can resume their itinerary.
Specifically, when an AMR reaches its last destination and it is marked as a `pickup`, the transportation request will be marked as complete and Open-RMF will begin publishing an `DispenserRequest` and wait for a matching `DispenserResponse`.
As soon as the transporter's `handle_signal` function receives the `pickup` signal, the matching `DispenserResponse` message will be published and the AMR will be released and complete its task.

#### Future work

* Move from using the building map to using the dry run feature of the task bidding system to assess feasibility and calculate task completion time estimates.
* Move from hardcoded `pickup` final phase to a `dropoff `.
* Implement workcells with multiple input / output stations and use them for transportation requests.
