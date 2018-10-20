# Lane following {#demo-lane-following status=ready}

This is the description of lane following demo.

Maintainer: Russell Buchanan

<div class='requirements' markdown="1">

Requires: Wheels calibration completed [wheel calibration](#wheel-calibration)

Requires: Camera calibration completed [Camera calibration](#camera-calib)

Requires: Joystick demo has been successfully launched [Joystick demo](#rc-control))

</div>

## Video of expected results {#demo-lane-following-expected}

[Video of demo lane following](https://drive.google.com/file/d/198iythQkovbQkzY3pPeTXWC8tTCRgDwB/view?usp=sharing)

## Duckietown setup notes {#demo-lane-following-duckietown-setup}

Assumption about Duckietown:

* A Duckietown with white and yellow lanes. No obstacles on the lane.
* Layout conform to Duckietown Appearance [Specifications](+opmanual_duckietown#duckietown-specs)
* Required tiles types: straight tile, turn tile
* Additional tile types:3-way/4-way intersection
* Configurated wireless network for communicating with Duckiebot.
* Good consistent lighting, avoid natural lighting.

## Duckiebot setup notes {#demo-lane-following-duckiebot-setup}

* Make sure the camera is heading ahead.
* Duckiebot in configuration [DB18-jwd](#duckiebot-configurations)


## Pre-flight checklist {#demo-lane-following-pre-flight}

* Turn on joystick (if applicable).
* Turn on battery of the duckiebot.
* Place duckiebot in lane so that enough of the lane markers are visible to the camera.
* Verify you can ping your duckiebot over the network.
* __IMPORTANT__ Make sure no containers are runing on the duckiebot which use either the camera or joystick. We will run these ROS nodes together in a new container.

## Demo instructions {#demo-lane-following-run}

### Step 1

Load the new container:

    laptop $ docker -H ![hostname].local run -it --net host --priviliged --v /data:/data --name lane_follower raabuchanan/rap-duckietown-controls:master18 /bin/bash

This will load the `lane_follower` container and ssh your terminal into the container. You will be at ![DUCKIEBOT_ROOT] which is probably `/home/software`.

### Step 2

Launch the lane follower with ROS;

    container $ roslaunch duckietown_demos lane_following.launch

Note: If ROS can't find the lanch file you may need to source the catkin workspace. try: `source ![DUCKIEBOT_ROOT]/catkin_ws/devel/setup.bash`

This launches several nodes and may take up to a minute to initialize everything. 

### Step 3

Now we will verify that `lane_filter_node` is working. On your laptop set your `ROS_MASTER_URI` to your duckiebot.

Ex.

    laptop $ export ROS_MASTER_URI=http://![hostname].local:11311/

Now with `rostopic list` we can see al lthe topics being published. Pro tip: use `rqt_graph` to get an overall picture of how all the nodes fit together. Elipses are nodes, small rectangles are topics and big rectangles are namespaces.

Let's take a look at the Anti-Instagram filter. This is the custom duckietown filter which detectes duckietown lanes.

    laptop $ rqt_image_view

Select the `/![hostname]/camera_node/image/compressed` topic from the drop down and you should see the video stream.

### Step 4

Run command:

    laptop $ rosparam set /![hostname]/line_detector_node/verbose true

so that line_detector_node will publish the image_with_lines.

Now select the `/![hostname]/line_detector_node/image_with_lines` in `rqt_image_view` and you should see something like this:

First, we show a [video](https://drive.google.com/open?id=1XDTNk8NgIlMEyC7R0vyqVm3TSj7Sowc8) of the expected behavior.

Et voilà! We are ready to drive around autonomously.


### Step 4

If you have a joystick you can skip this next command, otherwise we need to run the keyboard controller:

    laptop $ dts keyboard_control ![hostname]


|        Controls      | Joystick |  Keyboard |
|----------------------|:--------:|:---------:|
| Start Lane Following |  __R1__  |   __a__   |
| Stop Lane Following  |  __L1__  |   __s__   |
| Reset Anti-Instagram |  __X__   |   __i__   |


Start the lane following. The Duckiebot should drive autonomously in the lane. Intersections and red lines are neglected and the Duckiebot will drive across them like it is a __normal__ lane. Enjoy the demo.


## Troubleshooting {#demo-lane-following-troubleshooting}

### The duckiebot does not move

* Check if you can manually drive the duckiebot
    * Try re launching `dts keyboard control`
* Check if ROS messages are received on the robot on the `![hostname]/joy` topic

### The Duckiebot does not stay in a straight lane
* Check `rqt_image_view` and look at image_with_lines.
    * Check if you see enough segments. If not enough segments are visible, reset the Anti-Instagram filter.
    * Check if you see more segments and the color of the segments are according to the color of the lines in Duckietown
* Check your camera [calibrations](#camera-calib) are good.

### The Duckiebot does not drive nicely through intersections
This feature is not implemented for this demo. The duckiebot assumes only normal lanes during this demo.

### The Duckiebot cuts white line while driving on inner curves (avanced)

Solution (advanced):

Set alternative controller gains. While running the demo on the Duckiebot use the following to set the gains to the alternative values:


    laptop $ rosparam set /![hostname]/lane_controller_node/k_d -45

    laptop $ rosparam set /![hostname]/lane_controller_node/k_theta -11

Those changes are only active while running the demo and need to be repeated at every start of the demo if needed. If this improved the performance of your Duckiebot, you should think about permenantly change the default values in your catkin_ws.


## Demo failure demonstration {#demo-lane-following-failure}
The Anti-Instagram performs some self calibrations on startup, but if this happens when the robot cannot see the lane it will be poorly calibrated. This means it won't see enough lane segments, particularily around curves like in this [video](https://drive.google.com/open?id=1Hy6EjQ8QakfZliiSp_j2NV78_VpyPvCq).
To solve the problem Anti-Instagram needs to be relaunched. In the last part of the video the **X** button on the joystick is pressed and the Anti-Instagram node gets relaunched. We can see in RVIZ that the number of detected line segments gets increased drastically after the recalibration.
