# Lane following {#demo-lane-following status=ready}

This is the description of lane following demo.

<div class='requirements' markdown="1">

Requires: [Wheels calibration](#wheel-calibration) completed.

Requires: [Camera calibration](#camera-calib) completed.

Requires: [Joystick demo](#rc-control) has been successfully launched.

</div>

## Video of expected results {#demo-lane-following-expected}

[Video of demo lane following](https://drive.google.com/file/d/198iythQkovbQkzY3pPeTXWC8tTCRgDwB/view?usp=sharing)

## Duckietown setup notes {#demo-lane-following-duckietown-setup}

Assumption about Duckietown:

* A Duckietown with white and yellow lanes. No obstacles on the lane.
* Layout conform to Duckietown Appearance [Specifications](+opmanual_duckietown#dt-ops-appearance-specifications)
* Required tiles types: straight tile, turn tile
* Configurated wireless network for communicating with Duckiebot.
* Good consistent lighting, avoid natural lighting.

## Duckiebot setup notes {#demo-lane-following-duckiebot-setup}

* Make sure the camera is heading ahead.
* Duckiebot in configuration [DB18](#duckiebot-configurations)


## Pre-flight checklist {#demo-lane-following-pre-flight}

* Turn on joystick (if applicable).
* Turn on battery of the duckiebot.
* Place duckiebot in lane so that enough of the lane lines are visible to the camera.
* Verify you can ping your duckiebot over the network.
* __IMPORTANT__ Make sure no containers are running on the duckiebot which use either the camera or joystick. We will run these ROS nodes together in a new container.

## Demo instructions {#demo-lane-following-run}

### Step 1

Run the demo:

    laptop $ dts duckiebot demo --demo_name lane_following --duckiebot_name ![DUCKIEBOT_NAME] --package_name duckietown_demos

This will start the `demo_lane_following` container. You have to wait a while for everything to start working.

### Step 2

Now we will verify that `lane_filter_node` is working. On your laptop run `start_gui_tools`.

    laptop $ dts start_gui_tools ![hostname]

This will enter you into a container on your laptop that can talk to the ROS on the robot. To verify this do:

    laptop-container $ rostopic list

and you should see all of the rostopics listed there. If you see an output like "Cannot communicate with ROS_MASTER" that's a problem.

Next run:

    laptop-container $ rqt_image_view

Select the `/![hostname]/camera_node/image/compressed` topic from the drop down in the popup window and you should see the video stream.

### Step 3

Now we need to make `lane_filter_node` publish all the image topics.
We can do this by setting the ROS parameter `verbose` to `true`:

    laptop-container $ rosparam set /![hostname]/line_detector_node/verbose true

so that `line_detector_node` will publish the image_with_lines.

Now select the `/![hostname]/line_detector_node/image_with_lines` in `rqt_image_view` and you should see something like this:

First, we show a [video](https://drive.google.com/open?id=1XDTNk8NgIlMEyC7R0vyqVm3TSj7Sowc8) of the expected behavior.

Et voilà! We are ready to drive around autonomously.


### Step 4

If you have a joystick you can skip this next command, otherwise we need to run the keyboard controller:

    laptop $ dts duckiebot keyboard_control ![hostname]

|        Controls      |  Joystick  |  Keyboard |
|----------------------|:----------:|:---------:|
| Start Lane Following |   __R1__   |   __a__   |
| Stop Lane Following  |   __L1__   |   __s__   |


Start the lane following. The Duckiebot should drive autonomously in the lane. Intersections and red lines are neglected and the Duckiebot will drive across them like it is a normal lane. Enjoy the demo.

## Troubleshooting {#demo-lane-following-troubleshooting}

### The duckiebot does not move

* Check if you can manually drive the duckiebot
  * Try re launching `dts duckiebot keyboard_control ![hostname]`
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
