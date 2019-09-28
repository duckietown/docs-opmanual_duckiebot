# Lane following {#demo-lane-following status=ready}

This is the description of lane following demo.

<div class='requirements' markdown="1">

Requires: [Wheels calibration](#wheel-calibration) completed.

Requires: [Camera calibration](#camera-calib) completed.

Requires: [Joystick demo](#rc-control) has been successfully launched.

</div>

## Video of expected results {#demo-lane-following-expected}

<div figure-id="fig:lane_following_vid">
    <figcaption>Outcome of the lane following demo.
    </figcaption>
    <dtvideo src='vimeo:334931570'/>
</div>

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

### Step 1: Start the demo containers

Running this demo requires almost all of the main Duckietown ROS nodes to be up and running. As these span 3 Docker images (`dt-duckiebot-interface`, `dt-car-interface`, and `dt-core`, we will need to start all of them.

First, start all the drivers in `dt-duckiebot-interface`:

    laptop $ dts duckiebot demo --demo_name all_drivers --duckiebot_name ![DUCKIEBOT_NAME] --package_name duckiebot_interface --image duckietown/dt-duckiebot-interface:daffy
    
Then, start the glue nodes that handle the joystick mapping and the kinematics:

    laptop $ dts duckiebot demo --demo_name all --duckiebot_name ![DUCKIEBOT_NAME] --package_name car_interface --image duckietown/dt-car-interface:daffy

Finally, we are ready to start the high-level pipeline for lane following:

    laptop $ dts duckiebot demo --demo_name lane_following --duckiebot_name ![DUCKIEBOT_NAME] --package_name duckietown_demos --image duckietown/dt-core:daffy

You have to wait a while for everything to start working. While you wait, you can check in Portainer if all the containers started successfully and in their logs for any possible issues.

### Step 2: Make your Duckiebot drive autonomously!

If you have a joystick you can skip this next command, otherwise we need to run the keyboard controller:

    laptop $ dts duckiebot keyboard_control ![DUCKIEBOT_NAME]


|        Controls      |  Joystick  |     Keyboard     |
|----------------------|:----------:|:----------------:|
| Start Lane Following |   __R1__   |   <kbd>a</kbd>   |
| Stop Lane Following  |   __L1__   |   <kbd>s</kbd>   |


Start the lane following. The Duckiebot should drive autonomously in the lane. Intersections and red lines are neglected and the Duckiebot will drive across them like it is a normal lane. You can regain control of the bot at any moment by stopping the lane following and using the (virtual) joystick. Resuming the demo is as easy as pressing the corresponding start button.

Et voilà! We are ready to drive around autonomously.

### Step 3: Visualize the detected line segments (optional)

This step is not neccessary but provides a nice visualization of the line segments that the Duckiebot detects. 

For that, we need to make `lane_filter_node` publish all the image topics.

We can do this by setting the ROS parameter `verbose` to `true`:

    container $ rosparam set /![DUCKIEBOT_NAME]/line_detector_node/verbose true

so that `line_detector_node` will publish the image_with_lines.

Now select the `/![DUCKIEBOT_NAME]/line_detector_node/image_with_lines` in `rqt_image_view` and you should see something like this:

<div figure-id="fig:line_detector">
    <figcaption>Outcome of the line detector node.
    </figcaption>
    <dtvideo src='vimeo:334931437'/>
</div>

### Step 4: Extras

Here are some additional things you can try:

* Get a [remote stream](#read-camera-data) of your Duckiebot.
* Try to change some of the ROS parameters to see how your Duckiebot's behavior will change. 


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

    laptop $ rosparam set /![DUCKIEBOT_NAME]/lane_controller_node/k_d -45

    laptop $ rosparam set /![DUCKIEBOT_NAME]/lane_controller_node/k_theta -11

Those changes are only active while running the demo and need to be repeated at every start of the demo if needed. If this improved the performance of your Duckiebot, you should think about permenantly change the default values in your `catkin_ws`.
