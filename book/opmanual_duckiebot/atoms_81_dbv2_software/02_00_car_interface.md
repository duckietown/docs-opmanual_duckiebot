# Car Interface DBV2 {#dbv2-car-interface status=beta}

The repository [`dt-car-interface-dbv2`](https://github.com/duckietown/dt-car-interface-dbv2) contains only one
node: the DBV2 kinematics node. This is the only node in `dt-car-interface` that is different for DBV2. The rest
of `dt-car-interface` is used unaltered on DBV2.

To create and run a `dt-car-interface-dbv2` container, use this command:

    $ docker -H ![duckiebot name].local run \
      --privileged -dit -v /data:/data --name car-interface-dbv2 --network=host \
      --restart unless-stopped duckietown/dt-car-interface-dbv2:daffy-arm32v7

## Launching

To launch all of the important drivers, use the launchfile `packages/car_interface_dbv2/launch/all.launch`.
You can use the following command to do so, from within a `dt-car-interface-dbv2` container:

    $ roslaunch car_interface_dbv2 all.launch veh:=$VEHICLE_NAME

This will launch all of the necessary drivers from `dt-car-interface`, as well as the DBV2 version of the
kinematics node.

## Kinematics Node

The DBV2 kinematics node performs the same role as the DB18 kinematics node: It transforms linear and angular
velocity setpoints from `car_cmd` messages into motor commands. However, because the DBV2 has an entirely different
model, we need a different kinematics node.

The DBV2 kinematics node has also been updated to use the data from the new sensors on the DBV2. Specifically,
it uses the encoder to perform closed-loop control of the linear velocity of the Duckiebot. This allows other
nodes to command a specific velocity, and be confident that the Duckiebot will actually achieve that velocity.

If the encoder is not present, then the kinematics node will fall back to the default feed-forward control
of speed.

## Parameters

 - `angle_lim`: Maximum steering angle of the front wheels, in radians
 - `axis-distance`: Length of one axel of the DBV2
 - `baseline`: Distance from front axel to back axel
 - `cog_distance`: Distance from the front axel to the center of gravity of the Duckiebot
 - `limit`: Limit of the output to the DC motor that drives the rear wheels
 - `omega_max`: Maximum angular velocity, in radians / second
 - `radius`: Radius of the rear wheels, in meters
 - `trim`: Amount to add to steering to correct for slight imperfections in the steering mechanism
 - `v_max`: Maximum linear velocity of the Duckiebot, in meters / second
 - `k_wheel`: Motor constant, which maps the input to the motor (from -1.0 to +1.0) to the output speed
   of the motor.
 - `k_P`: P constant for the PI controller for the linear velocity of the Duckiebot
 - `k_I`: I constant for the PI controller for the linear velocity of the Duckiebot