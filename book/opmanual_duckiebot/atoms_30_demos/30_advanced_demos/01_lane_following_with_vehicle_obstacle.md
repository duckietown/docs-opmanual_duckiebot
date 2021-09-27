# Lane following with vehicle avoidance (LFV) {#demo-lane-following-vehicleobstacle status=ready}

This is the description of lane following with vehicle avoidance demo.

<div class='requirements' markdown="1">

Requires: [Wheels calibration](#wheel-calibration) completed.

Requires: [Camera calibration](#camera-calib) completed.

Requires: [Joystick demo](#rc-control) has been successfully launched.

Requires: [Lane Following demo](#demo-lane-following) gas veeb successfully completed

</div>

<!-- ## Video of expected results {#demo-lane-following-expected}

<div figure-id="fig:lane_following_vid">
    <figcaption>Outcome of the lane following demo.
    </figcaption>
    <dtvideo src='vimeo:334931570'/>
</div> -->

TODO: Add LFV result video

## Duckietown setup notes {#demo-lane-following-vehicleobstacle-duckietown-setup}

Assumption about Duckietown:

* A Duckietown with white and yellow lanes. No obstacles on the lane.
* Layout conform to Duckietown Appearance [Specifications](+opmanual_duckietown#dt-ops-appearance-specifications)
* Required tiles types: straight tile, turn tile
* Configurated wireless network for communicating with Duckiebot.
* Good consistent lighting, avoid natural lighting.

## Duckiebot setup notes {#demo-lane-following-vehicleobstacle-duckiebot-setup}

* Make sure the camera is heading ahead.
* Duckiebot in configuration [DB18](#duckiebot-configurations) or [DB19]()
* Make sure the vehilce obstalce has the circle pattern installed.


## Pre-flight checklist {#demo-lane-following-vehicleobstacle-pre-flight}

* Turn on joystick (if applicable).
* Turn on battery of the duckiebot.
* Place duckiebot in lane so that enough of the lane lines are visible to the camera.
* Verify you can ping your duckiebot over the network.
* __IMPORTANT__ Make sure no containers are running on the duckiebot which use either the camera or joystick. We will run these ROS nodes together in a new container.
* Make sure your duckiebot can perform lane following. 

## Demo instructions {#demo-lane-following-vehicleobstacle-run}

### Start the demo containers

Running this demo requires almost all of the main Duckietown ROS nodes to be up and running. As these span 3 Docker images (`dt-duckiebot-interface`, `dt-car-interface`, and `dt-core`). The `dt-duckiebot-interface` and `dt-car-interface` container typically starts with robot startup. You will need to start `dt-core` manually. 

First, check to make sure that `dt-duckiebot-interface` and `dt-car-interface` are running on your duckiebot via portainer, if not, do:

    laptop $ dts duckiebot demo --demo_name duckiebot-interface --duckiebot_name ![DUCKIEBOT_NAME] --package_name duckiebot_interface --image duckietown/dt-duckiebot-interface:daffy-arm32v7

    laptop $ dts duckiebot demo --demo_name car-interface --duckiebot_name ![DUCKIEBOT_NAME] --package_name car_interface --image duckietown/dt-car-interface:daffy-arm32v7

Then, we are ready to start the high-level pipeline for lane following:

    laptop $ dts duckiebot demo --demo_name multi_lane_following --duckiebot_name ![DUCKIEBOT_NAME] --package_name duckietown_demos

You have to wait a while for everything to start working. While you wait, you can check in Portainer if all the containers started successfully and in their logs for any possible issues.

### Make your Duckiebot drive autonomously!

If you have a joystick you can skip this next command, otherwise we need to run the keyboard controller:

    laptop $ dts duckiebot keyboard_control ![DUCKIEBOT_NAME]

|        Controls      |  Joystick  |     Keyboard     |
|----------------------|:----------:|:----------------:|
| Start Lane Following |   __R1__   |   <kbd>a</kbd>   |
| Stop Lane Following  |   __L1__   |   <kbd>s</kbd>   |


Start the lane following. The Duckiebot should drive autonomously in the lane. Intersections and red lines are neglected and the Duckiebot will drive across them like it is a normal lane. You can regain control of the bot at any moment by stopping the lane following and using the (virtual) joystick. Resuming the demo is as easy as pressing the corresponding start button.

Et voilà! We are ready to drive around autonomously.

### Expected outcome

The expected result for the lane following with vehicle avoidance is that the duckiebot will blink its tail lights yellow (greenish yellow) when it gets close to the vehicle in the way, while slowing down. At a certain distance, the duckiebot will stop completely and blink its tail light red to signal other duckiebots behind that there is a road anomaly.

### Visualize the detected vehicle (optional)

This step is not neccessary but provides a nice visualization of the line segments that the Duckiebot detects. 

In order to visualize the duckiebot's vehicle detection you can use the `rqt_image_view` image tool and select the vehicle detection output. For more information about `rqt_image_view` tools, you can find them [here](#view-image-using-rqt-image-view)

### Extras

Here are some additional things you can try:

* Try to change some of the ROS parameters to see how your Duckiebot's stopping behaviour will change. 
* Try to implement a new way of detecting the duckiebot in the way.

## Troubleshooting {#demo-lane-following-vehicleobstacle-troubleshooting}

### Generic Lane Following Problems

Generic lane following problems can happen, such as: 

 - The duckiebot does not move
 - The Duckiebot does not stay in a straight lane
 - The Duckiebot does not drive nicely through intersections
 - The Duckiebot cuts white line while driving on inner curves

For these problems, please refer to the duckiebot lane following tutorial [here](#demo-lane-following-troubleshooting) for troubleshooting instructions

### The Duckiebot does not stop behind the vehicle obstacles

Sometimes due to processing latency, the duckiebot will not be able to see the vehicle in front in time. This can also sometimes happen in the real world autonomous vehicles. It is an active research area and feel free to propose better solutions! Currently we use CV tools and the circuilar pattern to identify and calculate the distance between duckiebots.

To make sure the problem is not caused by the pipeline, you can also use the `rqt_image_view` tool described above to make sure the duckiebot is not failing the circular pattern detection. You will also observe the latency of the image detection through there. 