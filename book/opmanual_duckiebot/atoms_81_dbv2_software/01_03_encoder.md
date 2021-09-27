# Wheel Encoder Driver {#dbv2-encoder-driver status=beta}

The following files are important for the encoder driver:

 - `packages/sensor_suite/src/encoder_node.py`: This is the ROS node.
 - `packages/sensor_suite/config/encoder_node/default.yaml`: Default parameters for ROS node

## Overview

The encoder node uses the [pigpio library](http://abyz.me.uk/rpi/pigpio/) to read the encoder. This requires
that the pigpio daemon be started. In the container's `launch.sh` script, this is taken care of. However, if
you want to use this driver in a different context, make sure to start the pigpio daemon using the command `pigpiod`.

The driver sets up an interrupt handler which is triggered on either a falling or rising edge on the encoder GPIO line.
This interrupt handler simply keeps a count of the total number of edges since boot. Then, another thread
checks this count at regular intervals, calculates the average speed since the last check, and publishes the
result to the ROS topic `/![duckiebot name]/encoder_node/encoder_velocity`.

An alternative would be to publish the ROS topic once for each tick of the encoder. However, the first method
was chosen for the following reasons:

 - It guarantees that the ROS topic is published at a constant speed
 - If the wheel is not moving, the ROS topic is still published, indicating 0 speed

The prototype encoder used for testing has only 10 counts / revolution, which limits the resolution of this node.
At a polling frequency of 5Hz, and at maximum wheel speed, there are 30 encoder counts per second, or 6 per polling
period. Thus, at any motor speed between 0 and 100%, there are only 7 possible speeds that can be reported
by the encoder node. In testing, this proved to be sufficient, but only barely: Increasing the polling frequency
from 5 Hz to 10 Hz made the data too noisy to be useful.

Because there is only 1 GPIO line for the encoder, it cannot detect direction of rotation. Any node that uses
this data should also use the relevant `car_cmd` topic to determine which direction the wheels are expected to
be turning.

## Parameters

 - `polling_hz`: This is the frequency at which to read the current global encoder count, calculate the velocity,
   and publish the ROS topic. For the prototype encoder with 10 counts / revolution, 5 Hz is recommended as the
   maximum. A lower value will make the encoder node average the speed over a longer period, meaning less noise and
   higher resolution, but slower response to changes in speed.
 - `pin_encoder`: The GPIO pin of the Raspberry Pi to which the encoder is connected. On the Duckietown HUT v3.0,
   this is pin 19.
 - `radius`: Radius of the wheel. This is used to convert rotational velocity to linear velocity of the Duckiebot.
 - `holes_per_round`: Number of encoder counts per revolution. An encoder with more of these will provide higher
   resolution speed measurements, as described above.
 
## Calibration

The encoder node does not have a calibration procedure like the other sensor nodes. Instead, it should be
calibrated by measuring the diameter of the wheel of the Duckiebot and counting the number of encoder counts
per revolution, and providing those values in their respective parameters, described above.