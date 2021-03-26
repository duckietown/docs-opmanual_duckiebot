# Line follower lane controller {#dbv2-core-line-follower status=beta}

The `line_follower_lane_controller` package contains the lane controller node which uses the line follower sensors.
This lane controller node is a drop-in replacement for the original lane controller node, except that it does not
use the camera at all.

## Overview

This node uses a PI controller for steering, and maintains a constant forward velocity.

## Parameters

 - `steering_gain`: `P` constant for the steering PI controller
 - `k_I`: `I` constant for the steering PI controller
 - `max_I`: The integral of the error will be limited to +/- this value. This prevents the integral from becoming
   too large and causing the Duckiebot to lose the white line.
 - `update_hz`: How fast to run the control loop. This should be no faster than the line follower sensor
   update frequency.
 - `drive_speed`: Forward drive speed, in m/s
 - `*_threshold`: A line follower reading below this value will read as white, and a higher value is black.
   Before changing this value, you should consider recalibrating the line follower sensors, as described in
   [](#dbv2-calib-lf).