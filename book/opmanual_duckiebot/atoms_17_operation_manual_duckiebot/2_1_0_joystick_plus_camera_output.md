# `DB17`: RC and camera, remotely {#rc-cam-launched-remotely status=ready}

Assigned: Andrea Censi

<div class='requirements' markdown='1'>

Requires: You can run the joystick demo remotely. The procedure is documented
in [](#rc-launched-remotely).

Requires: You can read the camera data from ROS. The procedure is documented in
[](#read-camera-data).

Requires: You know how to get around in Byobu. You can find the Byobu tutorial
in [](+software_reference#byobu).

Results: You can run the joystick demo from your laptop and see the camera
  image on the laptop.

</div>

## Assumptions

We are assuming that the joystick demo in [](#rc-launched-remotely) worked.

We are assuming that the procedure in [](#read-camera-data) succeeded.

We also assume that you terminated all instances of `roslaunch` with
<kbd>Ctrl</kbd>-<kbd>C</kbd>, so that currently there is nothing running in any
window.

<!--
Joystick + camera output in remote laptop


Make sure that your robot ![robot name] Duckiebot and your laptop <laptop> are connected to the same network.

laptop $ ping ![robot name].local

and on your duckiebot:

duckiebot $ ping <laptop>.local

If one or the other does not work, it is very unlikely that the following will work. Fix before proceeding.

In case you rebooted the duckiebot, please execute:
duckiebot $ sudo ntpdate -u us.pool.ntp.org
duckiebot $ ss -s
duckiebot $ source environment.sh
    duckiebot $ source set_ros_master.sh <robot-name> -->


## Terminal setup

On the laptop, this time create **four** Byobu windows.

See: A quick reference about Byobu commands is in [](+software_reference#byobu).

You will use the four windows as follows:

- In the first window, you will run the joystick demo, as before.
- In the second window, you will launch the nodes that control the camera.
- In the third window, you will launch programs to monitor the data flow.
- In the fourth window, you will use `rviz` to see the camera image.

TODO: Add figures

## First window: launch the joystick demo

In the first window, launch the joystick remotely using the same procedure in [](#roslaunch-joystick).

    laptop $ source environment.sh
    laptop $ roslaunch duckietown joystick.launch veh:=![robot name]

You should be able to drive the robot with the joystick at this point.

## Second window: launch the camera nodes

In the second window, we will launch the nodes that control the camera.

The launch file is called `camera.launch`:

    laptop $ source environment.sh
    laptop $ roslaunch duckietown camera.launch veh:=![robot name]

You should see the red led on the camera light up.

Comment: It is recommended to launch the joystick and the camera from onboard the robot after sshing in - LP

## Third window: view data flow

Open a third terminal on the laptop.

You can see a list of topics currently on the `ROS_MASTER` with the commands:

    laptop $ source environment.sh
    laptop $ export ROS_MASTER_URI=http://![robot name].local:11311/
    laptop $ rostopic list



You should see the following:

    /diagnostics
    /![robot name]/camera_node/camera_info
    /![robot name]/camera_node/image/compressed
    /![robot name]/camera_node/image/raw
    /![robot name]/joy
    /![robot name]/wheels_driver_node/wheels_cmd
    /rosout
    /rosout_agg

## Fourth window: visualize the image using `rviz`

Launch `rviz` by using these commands:

    laptop $ source environment.sh
    laptop $ source set_ros_master.sh ![robot name]
    laptop $ rviz

See also: For more information about `rviz`, see [](+software_reference#rviz).

In the `rviz` interface, click "Add" on the lower left, then the "By topic"
tag, then select the "Image" topic by the name

    /![robot name]/camera_node/image/compressed

Then click "ok". You should be able to see a live stream of the image from the
camera.

## Proper shutdown procedure

To stop the nodes: You can stop the node by pressing <kbd>Ctrl</kbd>-<kbd>C</kbd> on the terminal
where `roslaunch` was executed. In this case, you can use <kbd>Ctrl</kbd>-<kbd>C</kbd> in the
terminal where you launched the `camera.launch`.

You should see the red light on the camera turn off in a few seconds.

Note that the `joystick.launch` is still up and running, so you can still drive
the vehicle with the joystick.
