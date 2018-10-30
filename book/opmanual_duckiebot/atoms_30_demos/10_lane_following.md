# Lane following {#demo-lane-following status=beta}

This is the description of lane following demo.

<div class='requirements' markdown="1">

Requires: Wheels calibration completed [wheel calibration](#wheel-calibration)

Requires: Camera calibration completed [Camera calibration](#camera-calib)

Requires: Joystick demo has been successfully launched [Joystick demo](#rc-control))

Requires: Duckiebot in configuration [DB17-jwd](#duckiebot-configurations)

Requires: [Calibrating](#wheel-calibration) the gain parameter to **0.6**.

</div>

## Video of expected results {#demo-lane-following-expected}

[Video of demo lane following](https://drive.google.com/file/d/198iythQkovbQkzY3pPeTXWC8tTCRgDwB/view?usp=sharing)

## Duckietown setup notes {#demo-lane-following-duckietown-setup}

Assumption about Duckietown:

* A duckietown with white and yellow lanes. No obstacles on the lane.
* Layout conform to Duckietown Appearance [Specifications](+opmanual_duckietown#duckietown_parts)
* Required tiles types: straight tile, turn tile
* Additional tile types:3-way/4-way intersection
* Configurated Wifi network or Duckietown Wifi network

Environment of demo:

* Good lightning

## Duckiebot setup notes {#demo-lane-following-duckiebot-setup}

* Make sure the camera is heading ahead.
* Duckiebot in configuration [DB17-jwd](#duckiebot-configurations)
* [Calibrating](#wheel-calibration) the gain parameter to **0.6**.


## Pre-flight checklist {#demo-lane-following-pre-flight}

Check: Turn on joystick.
Check: Turn on battery of the duckiebot.
Check: Duckiebot drives correctly with joystick.

## Demo instructions {#demo-lane-following-run}

Step 1: On duckiebot, in /DUCKIERTOWN_ROOT/ directory, run command:

    duckiebot $ make demo-lane-following

Wait a while so that everything has been launched.
Drive around with joystick to check if connection is working.

Step 2: Press X to recalibrate Anti-Instagram.

Step 3: Start the autonomous lane following by pressing the **R1** button on joystick. to start autonomous.

Step 4: The Duckiebot should drive autonomously in the lane. Intersections and red lines are neglected and the Duckiebot will drive across them like it is a __normal__ lane. Enjoy the demo.

Step 5: Stop the autonomous driving by pressing **L1** button on the joystick and switch to joystick control.


## Troubleshooting {#demo-lane-following-troubleshooting}
### The Duckiebot does not drive nicely across intersections
This is not a valid failure. The Duckiebot assumes only normal lanes during this demo. There is no module concerning the intersections. Therefore, weird behavior at intersections is expected and normal because there are no lines. The demo is only to demonstrate the lane following.

### The Duckiebot does not drive nicely in the lane
Solution 1:

Step 1: Turn on line segments.

    laptop $ rosparam set /'robotname'/line_detector_node/verbose true

Step 2: Open rviz.

    laptop $ rviz

Step 3: Look at the .../image_with_lines image output. Apply the anti-instagram callibration by pushing the X button on the joystick. Check if you see enough segments. If not enough segments are visible, press X button on joystick for Anti-Instagram relaunch. Check if you see more segments and the color of the segments are according to the color of the lines in Duckietown

Solution 2:
* Check the extrinsic and intrinsic [calibration](#camera-calib)

### Demo does not compile

Solution:

* Run `what-the-duck` and follow instructions.

### Duckiebot drives not with joystick
Solution:

* Turn joystick on and off multiple times.
* Check if battery is powered on.

### The Duckiebot cuts white line while driving on inner curves (avanced)

Solution (advanced):

Set alternative controller gains. While running the demo on the Duckiebot use the following to set the gains to the alternative values:


    duckiebot $ rosparam set /robot_name/lane_controller_node/k_d -45

    duckiebot $ rosparam set /robot_name/lane_controller_node/k_theta -11

Those changes are only active while running the demo and need to be repeated at every start of the demo if needed. If this improved the performance of your Duckiebot, you should think about permenantly change the default values in your catkin_ws.


## Demo failure demonstration {#demo-lane-following-failure}

If Anti-Instagram is badly calibrated, the duckiebot will not see enough line segments. This is especially a problem in the curve and the duckiebot will leave the lane. An example of this failure can be seen in this [video](https://drive.google.com/open?id=1Hy6EjQ8QakfZliiSp_j2NV78_VpyPvCq) for which we had a bad Anti-Instagram calibration. Hence, the Duckiebot sees not enough line segments and the lane following fails in the curve. To solve the problem Anti-Instagram needs to be relaunched. In the last part of the video the **X** button on the joystick is pressed and the Anti-Instagram node gets relaunched. We can see in RVIZ that the number of detected line segments gets increased drastically after the recalibration.
