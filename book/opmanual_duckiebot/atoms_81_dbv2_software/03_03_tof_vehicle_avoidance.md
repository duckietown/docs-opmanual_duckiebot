# ToF Vehicle Detection {#dbv2-core-tof-vehicle-avoidance status=beta}

The `tof_vehicle_detection` package contains the vehicle avoidance control node which uses the ToF sensors.
This node is a drop-in replacement for the original vehicle avoidance control node, except that it does not
use the camera at all.

## Overview

This node uses the front ToF distance sensors to detect the presence of a vehicle in front of the Duckiebot.
It then controls the speed of the Duckiebot to try to maintain a constant distance to the other vehicle.

To decide how to use the measurements from the 3 different front sensors, the node performs a weighted average.
When the lane controller node demands a turn, the ToF vehicle avoidance node will place more weight on the
sensor pointed in that direction.

This node uses a PID loop to maintain the desired distance from the vehicle, and uses the existing lane control
node for steering. Therefore, there can be issues with stability if this node makes the duckiebot move faster
or slower than the lane control node expects.

## Parameters

 - `desired_distance`: Distance in meters to keep behind another vehicle
 - `detection_min_distance`: If the ToF sensors detect something closer than this many meters, then the node will
   consider it a certain vehicle detection
 - `detection_max_distance`: If the ToF sensors detect something further than this many meters, then the node will
   ignore it.
 - `distance_avg`: Perform the rolling average over this many distance measurements
 - `omega_min`: Steering angle at which the node begins to use outer sensor
 - `omega_max`: Steering angle at which the node stops using the center sensor
 - `Kp`: `P` constant in the speed PID loop
 - `Ki`: `I` constant in the speed PID loop
 - `Kd`: `D` constant in the speed PID loop
 - `Kp_delta_v`: Unused
 - `max_speed`: Keep the speed of the Duckiebot below this many meters / second, even when trying to follow a vehicle.
   