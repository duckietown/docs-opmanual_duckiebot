# Tackling Lane Following with Vehicles using Semantic Segmentation {#demo-semantic-segmentation status=beta}

This is the description of the "Lane Following with Vehicles (LFV) using Semantic Segmentation" demo.

<div class='requirements' markdown="1">

Requires: Fully set up Duckiebot.

Results: A Duckiebot capable of performing lane following with other vehicles. The Duckiebot should stop moving if the other vehicles are too close to the Duckiebot.

</div>

## Video of expected results {#demo-semantic-segmentation-expected}

<figure>
    <figcaption>This video demonstrates the expected LFV results in the simulation. Top-left: camera image. Top-right: predicted segmentation map. Bottom-left: result of line fitting using RANSAC in image space (note that we only consider the bottom-half of the image). Bottom-right: visualization of the ground-projected points in Duckiebot's coordinate frame, where the color blue corresponds to the predicted follow point and other colors correspond to the segmentation classes. The segmentation classes are: (1) YELLOW = yellow lines, (2) WHITE = white lines, (3) PURPLE = red lines, (4) RED = duckiebot, (5) GREEN = static obstacles (such as duckies, cones, and barricade), and (6) BLACK = everything else. The Duckiebot drives autonomously using a pure pursuit controller. As seen in the video, the Duckiebot stops when the other vehicle gets too close.</figcaption>
    <img style='width:16em' src="sim.gif"/>
</figure>

<figure>
    <figcaption>This video demonstrates the expected LFV results in the real world. Left: camera image. Right: predicted segmentation map. The Duckiebot drives autonomously using a pure pursuit controller. The other Duckiebot with a duckie on top of its body was manually controlled. As seen in the video, the Duckiebot stops when the other vehicle gets too close.</figcaption>
    <img style='width:16em' src="real.gif"/>
</figure>

<figure>
    <figcaption>This video demonstrates the expected Lane Following (LF) without other vehicles results in the real world from a third person point of view. The Duckiebot drives autonomously using a pure pursuit controller and finishes one lap in 40 seconds.</figcaption>
    <img style='width:16em' src="lf.gif"/>
</figure>

<figure>
    <figcaption>This video demonstrates the expected LFV results in the real world from a third person point of view. The Duckiebot drives autonomously using a pure pursuit controller. The other Duckiebot with a duckie on top of its body was manually controlled. As seen in the video, the Duckiebot stops when the other vehicle gets too close.</figcaption>
    <img style='width:16em' src="lfv.gif"/>
</figure>


## Duckietown setup notes {#demo-indefinite-navigation-duckietown-setup}

To run this demo, you can setup a quite complex Duckietown. The demo supports normal road tiles, intersections and traffic lights. That makes it a a level more difficult than the [lane following demo](#demo-lane-following). Make sure that your Duckietown complies with the appereance specifications presented in [the Duckietown specs](+opmanual_duckietown#dt-ops-appearance-specifications). In particular correct street signaling is key to success of intersections handling.


## Duckiebot setup notes {#demo-indefinite-navigation-duckiebot-setup}

One (or possibly more) Duckiebot in configuration [DB18](#duckiebot-configurations).

## Pre-flight checklist {#demo-indefinite-navigation-pre-flight}

Check that every Duckiebot has sufficient battery charge and that they are all properly calibrated.

## Demo instructions {#demo-indefinite-navigation-run}

### Start the demo containers

Running this demo requires almost all of the main Duckietown ROS nodes to be up and running. As these span 3 Docker images (`dt-duckiebot-interface`, `dt-car-interface`, and `dt-core`, we will need to start all of them.

Warning: First, make sure all old containers from the images `dt-duckiebot-interface`, `dt-car-interface`, and `dt-core` are stopped. These containers can have different names, instead look at the image name from which they are run.

Then, start all the drivers in `dt-duckiebot-interface`:

    laptop $ dts duckiebot demo --demo_name all_drivers --duckiebot_name ![DUCKIEBOT_NAME] --package_name duckiebot_interface --image duckietown/dt-duckiebot-interface:daffy
    
Start also the glue nodes that handle the joystick mapping and the kinematics:

    laptop $ dts duckiebot demo --demo_name all --duckiebot_name ![DUCKIEBOT_NAME] --package_name car_interface --image duckietown/dt-car-interface:daffy

Finally, we are ready to start the high-level pipeline for indefinite navigation:

    laptop $ dts duckiebot demo --demo_name indefinite_navigation --duckiebot_name ![DUCKIEBOT_NAME] --package_name duckietown_demos --image duckietown/dt-core:daffy

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

### Extras

Here are some additional things you can try:

* Get a [remote stream](#read-camera-data) of your Duckiebot.
* You can visualize the detected line segments the same way as for the [lane following demo](#demo-lane-following)
* Try to change some of the ROS parameters to see how your Duckiebot's behavior will change. 

## Troubleshooting

Symptom: The Duckiebot fails at intersections.

Resolution: This problem is an open development problem, to improve the results tune the parameters of the `unicorn_intersection_node`, the procedure is explained in the [troubleshooting section](#trouble-unicorn_intersection).

Maintainer: Contact Gianmarco Bernasconi (ETHZ) via Slack for further assistance.
