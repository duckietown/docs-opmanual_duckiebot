# Indefinite Navigation {#demo-indefinite-navigation status=beta}

This is the description of the indefinite navigation demo.

<div class='requirements' markdown="1">

Requires: Wheels calibration completed.[wheel calibration](#wheel-calibration)

Requires: Camera calibration completed.[Camera calibration](#camera-calib)

Requires: Joystick demo has been successfully launched.[Joystick demo](#rc-control))

Requires: Fully set up Duckietown (including April tags for intersections)

Requires: A motor gain of approximately 0.65 (strong influence in open-loop intersections)
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

A Duckietown with white and yellow lanes. No obstacles on the lane. Red stop lines at intersections. If several Duckiebots are present while running this demo, LEDs need to be installed for explicit communication and coordination of Duckiebots at intersections.

## Duckiebot setup notes {#demo-indefinite-navigation-duckiebot-setup}

Make sure the camera is securely tightened and properly calibrated.

## Pre-flight checklist {#demo-indefinite-navigation-pre-flight}

Check: Joystick is turned on.

Check: Sufficient battery charge of the Duckiebot.

Check: Gain is set to approximately 0.65.

## Demo instructions {#demo-indefinite-navigation-run}

Follow these steps to run the indefinite navigation demo on your Duckiebot:

Step 1: On the Duckiebot, navigate to the `/DUCKIETOWN_ROOT/` directory, run the command:

    duckiebot $ make indefinite-navigation

Wait until everything has been launched. Press X to start anti-instagram. Pressing R1 will start the autonomous lane following and L1 can be used to revert back to manual control.

In the current open-loop intersection navigation, no Duckiebot will successfully run the demo the first time. Parameter tuning is a must. The only two parameters that should be adjusted are the gain and trim, previously defined during the [wheel calibration procedure](#wheel-calibration).

The parameter pair which makes your bot go straight will unlikely work for the lane following due to the current controller design. Start with your parameter pair obtained from wheel calibration. If your Duckiebot stays too long on a curve during crossing an intersection, decrease your gain in steps of 0.05. If the Duckiebot doesn't make the turn enough long, increase your gain in steps of 0.05.

Command to modify your gain (in this example to 0.65):

    &#36; rosservice call /![robot name]/inverse_kinematics_node/set_gain -- 0.65

Question: on laptop or bot?

Everything below is helpful for debugging if your robot does not follow the lane at all.

Step 2: Navigate to the Duckietown folder:

    laptop &#36; cd ~/duckietown

then source the environment:

    laptop &#36; source environment.sh

set the the ROS master to your vehicle:

    laptop &#36; source set_ros_master.sh ![robot name]

and finally launch rviz:

    laptop &#36; rviz

In rviz, two markerarrays:

- `/![robot name]/duckiebot_visualizer/segment_list_markers`, and
-  `/![robot name]/lane_pose_visualizer_node/lane_pose_markers`

can be visualized. The green arrow representing the pose estimate of the robot has to be in a reasonable direction.

Step 3: Always on the laptop, run:

    laptop &#36; rqt

In rqt, the images can be visualized are:

-  `/![robot name]/camera_node/image/compressed`,
-  `/![robot name]/line_detector_node/image_with_lines`,
-  `/![robot name]/lane_filter_node/belief_img`.

## Troubleshooting 

Maintainer: Contact Julien Kindle (ETHZ) via Slack for further assistance.
