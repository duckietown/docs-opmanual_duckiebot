# Line Following Sensor Driver {#dbv2-line-follower-driver status=beta}

The following files are important for the line follower driver:

 - `packages/sensor_suite/src/line_following_node.py`: This is the ROS node.
 - `packages/sensor_suite/src/line_following_calibration_node.py`: 
   This is used for calibration. See [](#dbv2-calib-lf)
 - `packages/sensor_suite/include/sensor_suite/line_following_sensor/`: Low-level driver Python package
 - `packages/sensor_suite/config/line_following_node/default.yaml`: Default parameters for ROS node

## Overview

At startup, the line follower node will check the ADC diagnostic register to determine whether the line following
sensors are present. If there is any error on the ADC, the node will gracefully shut down.

The ROS node reads from the line following sensors at a regular frequency, as set by the parameter `polling_hz`.
It then takes these raw values and adjusts them per the calibration, as described below.

These calibrated values are then published to the ROS topic `/![duckiebot name]/line_following_node/line_follower`. 

## Parameters

 - `polling_hz`: This is the frequency at which to read data from the line following sensors and publish it
   as a ROS topic. This value has been tested and verified to work up to 100Hz. However, the theoretical upper
   limit is much higher, but depends upon the data rates of other sensors.
 - `*.m`: This is the individual scaling factor applied to each sensor, as per the equation below.
 - `*.b`: This is the offset applied to each sensor, as per the equation below.
 
## Calibration

The calibration procedure runs as a ROS node. It should _not_ be run at the same time as the driver node.
It will ask the user to place the Duckiebot over different colors on a city tile, and take a series of measurements.
Then, it calculates a scale and offset to apply to each sensor individually, using the following formula:

    reported_value = scale * measured_value + offset

This scale and offset is determined based on an ideal brightness value for the colors in the city:

<col2 class="labels-col1" figure-id="tab:ideal-color-measures" 
        figure-caption="Ideal measurements for line follower sensors">
    <span>Yellow</span>
    <span>0.2</span>
    <span>White</span>
    <span>0.6</span>
    <span>Black</span>
    <span>1.0</span>
</col2>

The scale and offset values are stored in the file 
`/data/config/calibrations/sensor_suite/line_follower/![duckiebot name].yaml`. On startup, the line following driver
node will check this file for the parameters `*.m` and `*.b` (One of each for each of the four sensors).
