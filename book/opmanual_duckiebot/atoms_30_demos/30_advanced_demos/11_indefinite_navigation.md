# Indefinite Navigation {#demo-indefinite-navigation status=ready}

This is the description of the indefinite navigation demo.

<div class='requirements' markdown="1">

Requires: [Wheel calibration](#wheel-calibration) completed.

Requires: [Camera calibration](#camera-calib) completed.

Requires: Fully set up Duckietown.

Results: One or more Duckiebot safely navigating in Duckietown.

</div>

## Video of expected results {#demo-indefinite-navigation-expected}

<!--
[link 1 of lane following](https://photos.google.com/share/AF1QipMEwYvBW5hl3_l4M0f9on3RSKJmYftbWxo0nSyW7EMTBWs7iXRc_fHEc5mouSMSxA/photo/AF1QipPOmXr0yu__d_J0Wefp1Gm6sNTtptUk57FvS6Fo?key=M1ZWc2k0Nnl4ckFjd3dwRmV0WmdMSzFWU0xmOXh3)
-->

<div figure-id="fig:demo-indefinite-navigation-video">
    <figcaption>Demo: indefinite navigation
    </figcaption>
    <dtvideo src='vimeo:247596730'/>
</div>

TODO: add a different video with an up to specification Duckietown.

## Duckietown setup notes {#demo-indefinite-navigation-duckietown-setup}

To run this demo, you can setup a quite complex Duckietown. The demo supports normal road tiles, intersections and traffic lights. That makes it a a level more difficult than the [lane following demo](#demo-lane-following). Make sure that your Duckietown complies with the appereance specifications presented in [the Duckietown specs](+opmanual_duckietown#dt-ops-appearance-specifications). In particular correct street signaling is key to success of intersections handling.

## Duckiebot setup notes {#demo-indefinite-navigation-duckiebot-setup}

One (or possibly more) Duckiebot in configuration [DB18](#duckiebot-configurations).

## Pre-flight checklist {#demo-indefinite-navigation-pre-flight}

Check that every Duckiebot has sufficient battery charge and that they are all properly calibrated.

## Demo instructions {#demo-indefinite-navigation-run}

### Start the demo containers

Running this demo requires almost all of the main Duckietown ROS nodes to be up and running. As these span 3 Docker images (`dt-duckiebot-interface`, `dt-car-interface`, and `dt-core`). The `dt-duckiebot-interface` and `dt-car-interface` container typically starts with robot startup. You will need to start `dt-core` manually.

First, check to make sure that `dt-duckiebot-interface` and `dt-car-interface` are running on your duckiebot via portainer, if not, do:

    laptop $ dts duckiebot demo --demo_name duckiebot-interface --duckiebot_name ![DUCKIEBOT_NAME] --package_name duckiebot_interface --image duckietown/dt-duckiebot-interface:daffy-arm32v7

    laptop $ dts duckiebot demo --demo_name car-interface --duckiebot_name ![DUCKIEBOT_NAME] --package_name car_interface --image duckietown/dt-car-interface:daffy-arm32v7

Then, we are ready to start the high-level pipeline for indefinite navigation:

    laptop $ dts duckiebot demo --demo_name indefinite_navigation --duckiebot_name ![DUCKIEBOT_NAME] --package_name duckietown_demos

You have to wait a while for everything to start working. While you wait, you can check in Portainer if all the containers started successfully and in their logs for any possible issues.

### Make your Duckiebot drive autonomously!

If you have a joystick you can skip this next command, otherwise we need to run the keyboard controller:

    laptop $ dts duckiebot keyboard_control ![DUCKIEBOT_NAME]

| Controls             | Joystick |   Keyboard   |
| -------------------- | :------: | :----------: |
| Start Lane Following |  **R1**  | <kbd>a</kbd> |
| Stop Lane Following  |  **L1**  | <kbd>s</kbd> |

Start the lane following. The Duckiebot should drive autonomously in the lane. Intersections and red lines are neglected and the Duckiebot will drive across them like it is a normal lane. You can regain control of the bot at any moment by stopping the lane following and using the (virtual) joystick. Resuming the demo is as easy as pressing the corresponding start button.

Et voilà! We are ready to drive around autonomously.

### Extras

Here are some additional things you can try:

- Get a [remote stream](#read-camera-data) of your Duckiebot.
- You can visualize the detected line segments the same way as for the [lane following demo](#demo-lane-following)
- Try to change some of the ROS parameters to see how your Duckiebot's behavior will change.

## Troubleshooting the intersection handling

If your Duckiebot does not yield satisfactory results in intersection navigation, you might want to tune the related parameters.
Basically the `unicorn_intersection_node` (long story for the name) is a mixture of open loop commands and a re-use of the lane filter.
During the intersection, namely when the Duckiebot is in the `FSM` state `INTERSECTION_CONTROL`, the color perception of lines is changed. As a simple example if the goal is to go straight, the red lines will be perceived as white, so that it will be possible to follow the right white line. On top of this there are a few open loop commands that are used to help the Duckiebot face the correct direction. These parameters are stored in

    Software/catkin_ws/src/00-infrastructure/duckietown/config/baseline/unicorn_intersection/unicorn_intersection_node/default.yaml

You can change them online (while the demo is running) by using:

     duckiebot-container $ rosparam set ![your_parameter] ![your value]

You can see all the parameters by running:

    duckiebot-container $ rosparam list

And check the value of a specific one using:

    duckiebot-container $ rosparam get ![param name]

The ones you might want to modify are the feed-forward parts, stored in `ff_left`, `ff_right` and `ff_straight`. These parameters modify the output $\omega$ (angular velocity, positive in counterclockwise direction) for the time given in `time_left_turn`, `time_straight_turn` and `time_right_turn`, which you might want to change aswell.

Maintainer: Contact Gianmarco Bernasconi (ETHZ), Frank (Chude) Qian (UofT) via Slack for further assistance.
