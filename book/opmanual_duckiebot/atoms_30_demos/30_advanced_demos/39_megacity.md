# Lane following {#demo-megacity status=draft}

This is the description of the megacity demo.

Maintainer: Gianmarco Bernasconi

<div class='requirements' markdown="1">

Requires: A bot capable of running all the basic demos.

Requires: A robotarium.

Results: The great megacity demp.

</div>

## Video of expected results {#demo-megacity-expected}

[Video of demo lane following](https://drive.google.com/file/d/198iythQkovbQkzY3pPeTXWC8tTCRgDwB/view?usp=sharing)

## Duckietown setup notes {#demo-megacity-duckietown-setup}

Assumption about Duckietown:

* A Duckietown with white and yellow lanes. No obstacles on the lane.
* Layout conform to Duckietown Appearance [Specifications](+opmanual_duckietown#dt-ops-appearance-specifications)
* Required tiles types: straight tile, turn tile, intersections
* Configurated wireless network for communicating with Duckiebot.
* Good consistent lighting, avoid natural lighting.

## Duckiebot setup notes {#demo-megacity-duckiebot-setup}

* Make sure the camera is heading ahead.
* Duckiebot in configuration [DB18-jwd](#duckiebot-configurations)


## Pre-flight checklist {#demo-megacity-pre-flight}

* Turn on joystick (if applicable).
* Turn on battery of the duckiebot.
* Place duckiebot in lane so that enough of the lane lines are visible to the camera.
* Verify you can ping your duckiebot over the network.
* __IMPORTANT__ Make sure no containers are running on the duckiebot which use either the camera or joystick. We will run these ROS nodes together in a new container.

## Demo instructions {#demo-megacity-run}

### Step 1

Load the new container:

    laptop $ docker -H ![hostname].local run -it --net host --memory="800m" --memory-swap="1.8g" --privileged -v /data:/data --name megacity_demo gibernas/rpi-duckiebot-megacity-demo-master18:master

This will start the `megacity_demo` container. You have to wait a while for everything to start working.


### Step 2

If you have a joystick you can skip this next command, otherwise we need to run the keyboard controller:

    laptop $ dts duckiebot keyboard_control ![hostname]

|        Controls      | Joystick |  Keyboard |
|----------------------|:--------:|:---------:|
| Start Lane Following |  __R1__  |   __a__   |
| Stop Lane Following  |  __L1__  |   __s__   |


Start the autonomous mode. The Duckiebot should drive autonomously in the lane. Enjoy the demo.

## Troubleshooting {#demo-megacity-troubleshooting}

### The duckiebot does not move

* Check if you can manually drive the duckiebot
  * Try re launching `dts keyboard control`
* Check if ROS messages are received on the robot on the `![hostname]/joy` topic

### The Duckiebot does not stay in a straight lane

* Check `rqt_image_view` and look at image_with_lines.
  * Check if you see enough segments. If not enough segments are visible, reset the Anti-Instagram filter.
  * Check if you see more segments and the color of the segments are according to the color of the lines in Duckietown
* Check your camera [calibrations](#camera-calib) are good.


### The Duckiebot cuts white line while driving on inner curves (avanced)

Solution (advanced):

Set alternative controller gains. While running the demo on the Duckiebot use the following to set the gains to the alternative values:

    laptop $ rosparam set /![hostname]/lane_controller_node/k_d -45

    laptop $ rosparam set /![hostname]/lane_controller_node/k_theta -11

Those changes are only active while running the demo and need to be repeated at every start of the demo if needed. If this improved the performance of your Duckiebot, you should think about permenantly change the default values in your catkin_ws.

## Demo failure demonstration {#demo-megacity-failure}

TODO: write
