# AMOD18 cSLAM {#demo-cslam status=draft}

This is the description of the cSLAM (Centralized Simultaneous Localization And Mapping) demo. This demo allows a Duckiebot to localize itself and at the same time to build a map while it moves around the city. The task is achieved by using the camera of the Duckiebot, together with watchtowers located along the path, to detect April tags attached to the tiles, to the traffic signs and to the Duckiebot itself.

<div class='requirements' markdown="1">

Requires: Watchtowers, Tiles, April tags (tag family=Tag36h11, size=6.5cm, border=1), Duckiebots (with april tags on top of them).

Requires: Wheels calibration completed. [Wheel calibration](#wheel-calibration)

Requires: Camera calibration completed. [Camera calibration](#camera-calib)

Requires: Joystick demo has been successfully launched. [Joystick demo](#rc-control)

Requires: ROS installation on local computer.

Requires: Docker installation on local computer.

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
  - April tags have to be placed on the tiles, traffic signs, and Duckiebots. These tags, should all be unique!
  - For a detailed map to be visualized, the poses of the April tags in the Duckietown must be (approximately) known beforehand.
  - Watchtowers have to be spread across the entire Duckietown. Preferably the combined field of view covers the entire Duckietown.
* Weather
  - Lighting has to be bright enough for the April tags to be seen clearly by camera.
  - Assumption: it is always sunny. Rain never occurs in Duckietown.

## Duckiebot setup notes {#demo-cslam-duckiebot-setup}

* All Duckiebots are required to have an April tag mounted on top of them. The bottom side of the April tag has to be pointing towards the rear of the Duckiebot. The center of the April tag should be aligned as closely as possible to the center of gravity of the Duckiebot.


## Pre-flight checklist {#demo-cslam-pre-flight}

Check: The Duckiebot has sufficient battery and is powered on.

Check: The `roscore`, `ros-picam`, and `joystick` containers are turned on.

Check: Keyboard control is launched for this Duckiebot.

Check: ROS is installed on your local computer. _Is this neccessary if everything is Dockerized?_

Check: Docker is installed on your local computer.

Check: The watchtowers are powered on

## Demo instructions {#demo-cslam-run}

### Step 0
Before starting, please install ROS Kinetic on your local computer by following the official installation instructions [here](http://wiki.ros.org/kinetic/Installation/Ubuntu). Please install the Desktop-Full version.

Please install Docker on your local computer by following the official installation instructions [here](https://docs.docker.com/install/linux/docker-ce/ubuntu/). It is recommended you also have [Docker-compose](https://docs.docker.com/compose/install/)

### Step 1: Set up the watchtowers. _You can skip this step for the Thursday demo_

To burn the SD card for each watchtower, the same instructions as for Duckiebots apply. [Duckiebot initialization](#setup-duckiebot)

The `hostname` of each watchtower should be of the form `demowatchtower*01*`, where `*01*` stands for two digits.
The containers `roscore` and `ros-picam` are required:

    laptop $ docker -H ![hostname].local run -d --privileged --name roscore --network=host -v /data:/data --restart always duckietown/rpi-ros-kinetic-roscore:master18
    laptop $ docker -H ![hostname].local run -d --name ros-picam --network=host --device /dev/vchiq -v /data:/data --restart always duckietown/rpi-duckiebot-ros-picam:master18

It is also necessary to pull the Docker image for the acquistion node:

    laptop $ docker -H ![hostname].local pull aleksandarpetrov/cslam-aquisition-rpi

TODO: move the image to duckietown?

### Step 2: Setup the April Tags _You can skip this step for the Thursday demo_
Print out the April tags and place them on top of Duckiebots and in Duckietown
    (Provide location for Benson to place pre-generated April tags)
    
### Step 3: Setup a ROS Master machine

### Step 4: Configure the watchtowers

In order to start processing data on the watchtowers you need to run the `cslam-aquisition` container. This requires that you know the hostname and the IP address of the machine serving as the ROS Master for this demo. Once you know the hostname you can get its IP address by pinging it.

We have made a `bash` script that allows to easily set up all the the watchtowers. You can find it in `duckietown-cslam/scripts/watchtowers_setup.sh`. You will need to edit the `SERVER_HOSTNAME` and `SERVER_IP` in this file to the ones of your ROS Master.
Then you can run it with `bash watchtowers_setup.sh`.

### Step 5: Test the watchtowers

Setup the diagnostics tool to check that the status of the watchtowers are `OK` where data was received in the last XX seconds. 

    laptop $ docker pull bensonkuan/cslam-diagnostics
    laptop $ xhost +local:`docker inspect --format='{{ .Config.Hostname }}' $containerId`
    laptop $ docker run -it --rm --net=host -e ROS_MASTER_IP=http://![rosmaster_name].local:11311 -e ROS_IP=![rosmaster_IP] bensonkuan/cslam-diagnostics
    
TODO: check ways for other computers to be rosmaster (it only allows local computer to be rosmaster now)

If some of the watchtowers does not appear in the list, then it was likely not configured properly. Sometimes this is due to connection issues. Try to repeat the previous step again.


### Step 6: Setup the Duckiebot

The Duckiebot should have the following 3 containers runnning:

- `roscore`
- `ros_picam`
- `joystick`

You can start them or check if they are already running via Portainer. 

TODO: Add links to the parts of the book where it's explained how to run these.

As the Duckiebot usually has other nodes running we spare it processing of images and odometry by offloading this to a computer. To do this we need to run the acquisition node container. The acquisition node has a lot of configuration parameters. That is where `docker-compose` is handy. You can check the example `.yml` file given: `duckietown-cslam/scripts/docker-compose-duckiebot-x86.yml`. Here you need to update a few lines:

- Replace `duckiebotHostname` in `acquisition_node_duckiebotHostname`, `ACQ_ROS_MASTER_URI_DEVICE=duckiebotHostname.local` and `ACQ_DEVICE_NAME=duckiebotHostname` with your Duckiebot's hostname.
- Replace `XXX.XXX.XXX.XXX` in `ACQ_ROS_MASTER_URI_DEVICE_IP` with your Duckiebot's IP address. You can get this if you ping it.
- Replace `ROS_MASTER_Hostname` in `ACQ_ROS_MASTER_URI_SERVER=ROS_MASTER_Hostname.local` with your ROS Master's hostname. You should have gotten this already when you configured the watchtowers. 
- Replace `XXX.XXX.XXX.XXX` in `ACQ_ROS_MASTER_URI_SERVER_IP` with your ROS Master's IP address. You should have gotten this already when you configured the watchtowers. 

You can then start the container by running:

`docker-compose -f docker-compose-duckiebot-x86.yml up`

Make the Duckiebot see an april tag and you should see that you receive messages from it in the Diagnostics tool.

### Step 7
Set up and run the visualization of the map, duckiebots, watchtowers, and traffic signs using the following commands:

    laptop $ docker pull surirohit/cslam-visualization
    laptop $ xhost +local:`docker inspect --format='{{ .Config.Hostname }}' $containerId`
    laptop $ docker run -it --rm --net=host -e ROS_MASTER_IP=http://![rosmaster_name].local:11311 -e ROS_IP=![rosmaster_IP] surirohit/cslam-visualization
    
TODO: check ways for other computers to be rosmaster (it only allows local computer to be rosmaster now)
### Step 8
Set up graph optimizer
    
    (TODO: Amaury)

### Step 9
Control the Duckiebot manually around Duckietown

    laptop $ dts duckiebot keyboard_control ![duckie_hostname]
    
Look at the diagnostic tool to ensure the messaging status of the Duckiebots are `OK` where data was received in the last 10 seconds. If the Duckiebot messages does not appear in the list, then it was likely not configured properly. Sometimes this is due to connection issues. 

### Step 10: Shut everything off

## Troubleshooting {#demo-cslam-troubleshooting}

### April tags printed may be of wrong size
Check that the printed April tags are of size 6.5cm as the printer might have done some scaling to the tags.

### Check Diagnostics tool 
Check that messages are received frequently. Is not device may be suffering from poor connection and will need to restart or configuration for device was done wrongly and it needs to be reconfigured again. Do check that the network signal is strong enough for the devices to communicate with one another.

## Demo failure demonstration {#demo-cslam-failure}

Finally, put here a video of how the demo can fail, when the assumptions are not respected.
