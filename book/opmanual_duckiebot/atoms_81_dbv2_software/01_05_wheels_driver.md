# DBV2 Wheels Driver Node {#dbv2-wheels-driver status=beta}

The wheels driver node for DBV2 is located in `dt-core-dbv2/packages/wheels_driver_dbv2`.

The wheels driver required the pigpio daemon to be running. This is taken care of in the `launch.sh` script
for the `dt-duckiebot-interface-dbv2` container, but if you run the wheel driver in another context, you will
need to first start the pigpio daemon with the command `pigpiod`.

## Overview

The wheels driver node functions similarly to the DB18 wheels driver node: It subscribes to the `wheels_cmd`
messages from the kinematics node, and physically controls the motors. However, the difference is that
it subscribes to the topic `/![duckiebot name]/wheels_driver_node/wheels_cmd_dbv2`, which uses messages
of the type `WheelsCmdDBV2Stamped`. This is necessary because the DBV2 uses one DC motor and one servo,
instead of 2 DC motors.

## Parameters

 - `wheel_distance`: Distance between the front and back axles of the car
 - `duty_limit`: Maximum duty cycle for the servo, in percentage. Use this value to prevent the
   servo from being overdriven to an angle which is physically prevented by the steering system.
 