# AMOD18 cSLAM {#demo-cslam status=draft}

This is the description of the cSLAM (Centralized Simultaneous Localization And Mapping) demo. This demo allows a Duckiebot to localize itself and at the same time to build a map while it moves around the city. The task is achieved by using the camera of the Duckiebot, together with watchtowers located along the path, to detect April tags attached to the tiles, to the traffic signs and to the Duckiebot itself.

<div class='requirements' markdown="1">

Requires: Watchtowers, Tiles, April tags (tag family=Tag36h11, size=6.5cm, border=1), Duckiebots.

Requires: Wheels calibration completed. [Wheel calibration](#wheel-calibration)

Requires: Camera calibration completed. [Camera calibration](#camera-calib)

Requires: Joystick demo has been successfully launched. [Joystick demo](#rc-control))

Requires: ROS installation on local computer.

Requires: The Duckiebot in configuration DB18 section B-11.

</div>

## Video of expected results {#demo-cslam-expected}

First, we show a video of the expected behavior (if the demo is successful).

## Duckietown setup notes {#demo-cslam-duckietown-setup}

Layout of Duckietown:

* Layout
  - Tiles should be arranged in such a way that the path forms a closed loop.
  - Traffic lights are good to have but not necessary (optional).
  - Any 2 watchtower should observe at least one common April tag and all the April tags should be observed by all the watchtowers.
* Infrastructure
  - April tags have to be placed on the tiles and traffic signs.
  - For a detailed map to be visualized, the poses of the April tags in the Duckietown must be known beforehand (optional).
  - Watchtowers have to be spread across the entire Duckietown. Preferably the combined field of view covers the entire Duckietown.
* Weather
  - Lighting has to be good for the April tags to be seen clearly by camera.
  - Assumption: it is always sunny. Rain never occurs in Duckietown.

## Duckiebot setup notes {#demo-cslam-duckiebot-setup}

* All Duckiebots are required to have an April tag mounted on top of them. The bottom side of the April tag has to be pointing towards the rear of the Duckiebot. The center of the April tag should be aligned as closely as possible to the center of gravity of the Duckiebot.


## Pre-flight checklist {#demo-cslam-pre-flight}

Check: the Duckiebot has sufficient battery.

Check: the `ros-picam` container is turned on.

Check: Joystick is turned on.

Check: ROS is installed on your local computer.

## Demo instructions {#demo-cslam-run}

### Step 0
Before starting, please install ROS on your local computer by following the official installation instructions [here](http://wiki.ros.org/kinetic/Installation/Ubuntu). Please install the Desktop-Full version.

### Step 1
Set up the watchtowers.
To burn the SD card for each watchtower, the same instructions as for Duckiebots apply. [Duckiebot initialization](#setup-duckiebot)

The `hostname` of each watchtower should be of the form `demowatchtower*01*`, where `*01*` stands for two digits.
The containers `roscore` and `ros-picam` are required:

    laptop $ docker -H ![hostname].local run -d --privileged --name roscore --network=host -v /data:/data --restart always duckietown/rpi-ros-kinetic-roscore:master18
    laptop $ docker -H ![hostname].local run -d --name ros-picam --network=host --device /dev/vchiq -v /data:/data --restart always duckietown/rpi-duckiebot-ros-picam:master18

It is also necessary to pull the Docker image for the acquistion node:

    laptop $ docker -H ![hostname].local pull aleksandarpetrov/cslam-aquisition-rpi

TODO: move the image to duckietown?

### Step 2
Print out the April tags and place them on top of Duckiebots and in Duckietown
    (Provide location for Benson to place pre-generated April tags)

### Step 3
Pull the Acquisition, Diagnostics, Simulator and Graph pose optimizer Docker images onto your laptop:

    laptop $ (fill up)

### Step 4
Run the pulled Docker images:

    laptop $ (fill up)

### Step 5
Control the Duckiebot manually around Duckietown

    laptop $ dts duckiebot keyboard_control ![duckie_hostname]


## Troubleshooting {#demo-cslam-troubleshooting}

### April tags printed may be of wrong size
Check that the print April tags are of size 6.5cm as the printer might have done some scaling to the tags.

## Demo failure demonstration {#demo-cslam-failure}

Finally, put here a video of how the demo can fail, when the assumptions are not respected.
