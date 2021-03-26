# Duckiebot Interface DBV2 {#dbv2-duckiebot-interface status=beta}

The repository [`dt-duckiebot-interface-dbv2`](https://github.com/duckietown/dt-duckiebot-interface-dbv2) contains
all of the low-level drivers needed for DBV2. It contains the sensor nodes, which read sensor data and publish it
to ROS topics, and a new wheels driver node, to account for the different motor configuration on DBV2.

This container should run _instead of_ `dt-duckiebot-interface`. It builds on top of `dt-duckiebot-interface`, and
contains all of the necessary original functionality. Therefore, make sure there is no `dt-duckiebot-interface`
container running on your DBV2. To start a `duckiebot-interface-dbv2` container, use the following command:

    $ docker -H ![duckiebot name].local run \
      --privileged -dit -v /data:/data --name duckiebot-interface-dbv2 --network=host \
      --restart unless-stopped duckietown/dt-duckiebot-interface-dbv2:daffy-arm32v7
      
The nodes `wheels_driver_dbv2` and `encoder_node` rely upon the pigpio library and associated pigpio daemon.
In the `launch.sh` script, this daemon is started automatically. If you use this container without using the launch
file, you should make sure you start the pigpio daemon by running the command `pigpiod`.

Each package and node in this container is documented in the following sections.

## Launching

To launch all of the important drivers, use the launchfile `packages/duckiebot_interface_dbv2/launch/all_drivers.launch`.
As the name implies, this will launch all of the drivers in `dt-duckiebot-interface-dbv2`. It will also launch
all drivers from `dt-duckiebot-interface` which are not overridden by a newer DBV2 version. You can use the
following command to do so, from within a `dt-duckiebot-interface-dbv2` container:

    $ roslaunch duckiebot_interface_dbv2 all_drivers.launch veh:=$VEHICLE_NAME

Both the software and hardware of DBV2 are designed to be flexible, such that any subset of sensors can be used.
Each sensor driver is designed to try and connect to its respective sensor, and shutdown gracefully if it fails.
Therefore, this launchfile simply starts all available sensor drivers, but the end result will be that only the
necessary drivers will be running.