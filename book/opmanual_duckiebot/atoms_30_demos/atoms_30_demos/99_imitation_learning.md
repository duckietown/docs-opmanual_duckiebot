# Lane control with supervised learning {#demo-imitation-learning status=beta}

This is the description of lane following demo.

<div class='requirements' markdown="1">

Requires: Wheels calibration completed.[wheel calibration](#wheel-calibration)

Requires: Camera calibration completed.[Camera calibration](#camera-calib)

Requires: Joystick demo has been successfully launched.[Joystick demo](#rc-control))

Requires: One Movidius Neural Compute Stick. The ETH team only has two sticks. If you want to try out, contact Yang Shaohui or Wang Tianlu.

Requires: Movidius Neural Compute Stick api installed on Duckiebot.[Install ncsdk](http://purl.org/dth/ncsdk-install).

Note: You do not need the whole ncsdk (which requires caffe and tensorflow to be installed on Duckiebot, which causes troubles and tons of time) to run this demo. Just installing the api will be enough.

</div>

## Video of expected results {#demo-imitation-learning-expected}

The videos have been recorded. See the following websites. [Recorded video](https://youtu.be/FCP8Ndoxae0)

## Duckietown setup notes {#demo-imitation-learning-duckietown-setup}

A duckietown with white and yellow lanes. No obstacles on the lane. Actually any kind of map could be handled by the pretrained CNN network. You can even build a map which is totally different from all the map formats we currently have.

## Duckiebot setup notes {#demo-imitation-learning-duckiebot-setup}

Make sure the camera is heading ahead. Tighten the screws if necessary.

Make sure the bot sees white lane on its right and yellow lane on its left.

Make sure the nerual stick keeps some distance from the USB port of the raspberry PI because both things will be heating. An example appearance of the Duckiebot can be found here. [](#fig:Duckie_appearance)

 <div figure-id="fig:Duckie_appearance" figure-caption="The Duckiebot appearance">
      <img src="duckiebot-appearance-super-learning.jpg" style='width: 15em'/>
 </div>


## Pre-flight checklist {#demo-imitation-learning-pre-flight}

Check: turn on joystick.

Check: Enough battery of the duckiebot.

## Demo instructions {#demo-imitation-learning-run}

Step 1: On duckiebot, checkout to the branch "devel-super-learning-jan15". Of course, do git pull first. Use 'catkin_make' to rebuild the src folder.

Step 2: On duckiebot, in /DUCKIERTOWN_ROOT/ directory, run command:

    duckiebot $ make demo-imitation-learning

Wait a while so that everything has been launched. Press R1 to start autonomous lane following computed by the Movidius Neural Stick. Press L1 to switch to joystick control. You can see the predictions of heading direction are printed out on the screen.

Step 3: On laptop, source the enviroment, ROS master and vehicle name file so that you can remotely watched the performance of the car. Run

    laptop $ rqt_graph

You will find that there are much less ros nodes and topics compared with the traditional lane following demo. An example ros node picture is available here. [](#fig:ros_graph)

 <div figure-id="fig:ros_graph" figure-caption="The Ros graph">
      <img src="rosgraph-super-learning.png" style='width: 15em'/>
 </div>


## Troubleshooting {#demo-imitation-learning-troubleshooting}

Contact Yang Shaohui(ETHZ) via Slack or Email(shyang@ethz.ch) if any trouble occurs.
