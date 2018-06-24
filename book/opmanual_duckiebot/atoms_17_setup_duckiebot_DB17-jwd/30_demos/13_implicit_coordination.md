# Implicit Coordination {#implicit-coordination status=beta}

This demo allows multiple Duckiebots that stop at an intersection to coordinate themself without explicit communication and clear the intersection.

<div class='requirements' markdown="1">

Requires: Duckiebot in configuration DB-wj

Requires: Camera calibration completed. [camera calibration](#camera-calib)

Requires: Wheel calibration completed. [wheel calibration](#wheel-calibration)

Requires: Object Detection setup.

</div>

## Video of expected results {#implicit-coordination-expected}

First, we show a video of the expected behavior (if the demo is succesful).

TODO: add video

## Duckietown setup notes {#implicit-coordination-duckietown-setup}
The Duckietown used for this demo has to be equipped with...

* at least one intersection.
* april tags providing information about the intersection type.

## Duckiebot setup notes {#implicit-coordination-duckiebot-setup}

No special setup is needed for the Duckiebot.

## Laptop setup notes {#implicit-coordination-laptop-setup}

Tensorflow needs to be installed:

  [Installing Tensorflow on Ubuntu](https://www.tensorflow.org/install/install_linux)

Inference model needs to be added:

Move to correct folder:

    laptop $ cd ~/duckietown/catkin_ws/src/80-deep-learning/object_detection/models

Download model from [here](https://drive.google.com/open?id=1mEvDk4db7xB7q_tNHkI6V-F4dHWDyCmU) and place in the folder  

## Pre-flight checklist {#implicit-coordination-checklist}

Check: Joystick is turned on.

Check: Sufficient battery charge of the Duckiebot.

## Demo instructions {#implicit-coordination-run}

Everything should be run from branch 'devel-implicit-coordination-intersection', don't forget to build again.

Step 1: Place the participating Duckiebots on the ways to the same intersection with enough distance to recognize stop lines.

Step 2: Run the following commands on the Duckiebots:

Make sure you are in the Duckietown folder:

    duckiebot $ cd ~/Duckietown

Run the demo:

    duckiebot $ make demo-implicit_coordination

Step 3: Run the following commands on the laptop:

Make sure you are in the Duckietown folder:

    laptop $ cd ~/Duckietown

Set up the environment:

    laptop $ source environment.sh

Set your duckiebot as the ROS master:

    laptop $ source set_ros_master.sh ![robot name]

Run multivehicle detection:

    laptop $ roslaunch duckietown multivehicle_detection.launch veh:=![robot name]

Step 4: Press R1 on your connected gamepad. The duckiebots now perform lane following until they stop at the intersection, where they coordinate themself and clear the intersection.

## Troubleshooting {#implicit-coordination-troubleshooting}

TODO: add troubleshooting

## Demo failure demonstration {#implicit-coordination-failure}

TODO: coming soon
