# AMOD18 cSLAM {#demo-cslam status=draft}

<div figure-id="fig:g2o_viewer" figure-caption="g2o_viewer window">
     <img src="cSLAM_images/cSLAM_logo.png" style='width: 20em'/>
</div>

This is the description of the cSLAM (Centralized Simultaneous Localization And Mapping) demo. This demo allows a Duckiebot to localize itself, while the watchtowers and Duckiebots work together to build a map of the arena. The task is achieved by using the camera of the Duckiebot, together with watchtowers located along the path, to detect AprilTags attached to the tiles, the traffic signs, and the Duckiebot itself.

<div class='requirements' markdown="1">

Requires: Watchtowers, Tiles, AprilTags (tag family=Tag36h11, size=6.5cm, border=1), Duckiebots (with AprilTags on top of them), Local (duckietown) wireless network.

Requires: Wheels calibration completed. [Wheel calibration](#wheel-calibration)

Requires: Camera calibration completed. [Camera calibration](#camera-calib)

Requires: Joystick demo has been successfully launched. [Joystick demo](#rc-control)

Requires: Two computers

Requires: ROS installation on at least one of the computers.

Requires: Docker installation on both computers. [Laptop setup](#laptop-setup) [Docker-compose](https://docs.docker.com/compose/install/) is also recommended to be installed but is not required.

Requires: The Duckiebots and watchtowers in configuration DB18 until [Section B-11](#wheel-calibration).

</div>

## Video of expected results {#demo-cslam-expected}

First, we show a video of the expected behavior (if the demo is successful).

## Demo code {#demo-code}

The main cSLAM repository is [here](https://github.com/duckietown/duckietown-cslam). When we refer to configuration files or scripts, they will be here. Clone it for easy access.

There are also more detailed about the code, different configuration parameters and how to maintain it.

## Introduction to cSLAM {#demo-introduction}

TODO: Put a description, illustrations and explanation of what cSLAM is, the different parts of it, how they communicate, what's the general philosophy, etc.

## Duckietown setup notes {#demo-cslam-duckietown-setup}
TODO: Most of this will go away and a reference to the duckietown specs is needed


Layout of Duckietown:

### Layout
  * Traffic lights are good to have but not necessary (optional).
  * Each floor AprilTag should have at least two watchtowers seeing it. It is recommended, but not necessary, that the bipartite graph of watchtowers and AprilTags are connected.
### Infrastructure
  * AprilTags have to be placed on the tiles, traffic signs, and Duckiebots. These tags should all be unique!
  * For a detailed map to be visualized, the poses of the AprilTags in the Duckietown must be (approximately) known beforehand.
  * Watchtowers have to be spread across the entire Duckietown. Preferably the combined field of view covers the entire Duckietown.
### Weather
  * Lighting has to be bright enough for the AprilTags to be seen clearly by camera.
  * Assumption: it is always sunny. Rain never occurs in Duckietown.

## Duckiebot setup notes {#demo-cslam-duckiebot-setup}

* All Duckiebots are required to have an AprilTag mounted on top of them.
* The bottom side of the AprilTag has to be pointing towards the rear of the Duckiebot.
* The center of the AprilTag should be aligned as closely as possible to the center of gravity of the Duckiebot.


## Pre-flight checklist {#demo-cslam-pre-flight}

Check: The Duckiebot has sufficient battery and is powered on.

Check: The `roscore`, `ros-picam`, `joystick`, and `keyboard_control` containers are turned on for each Duckiebot.

Check: ROS is installed on your local computer. TODO: _Is this neccessary if everything is Dockerized?_

Check: Docker is installed on your local computer. TODO: Should we mention this again?

Check: The watchtowers are powered on

## Demo instructions {#demo-cslam-run}

### Step 0: Preliminaries {#demo-cslam-run-0}
Before starting, please install ROS Kinetic on your local computer by following the official installation instructions [here](http://wiki.ros.org/kinetic/Installation/Ubuntu). Please install the Desktop-Full version.

Please install Docker on your local computer by following the official installation instructions [here](https://docs.docker.com/install/linux/docker-ce/ubuntu/). It is recommended you also have [Docker-compose](https://docs.docker.com/compose/install/)

Clone duckietown-cslam : https://github.com/duckietown/duckietown-cslam and follow readme instructions to install everything (make sure you install g2o for python2)

Make sure all devices you are using are connected to the same WiFi network (typically that would be `duckietown`)

TODO: Installation instructions probably not important here since they are prerequisites. Just docker-compose maybe. That should also be added to prerequisites in my opinion. Yes, the installation shouldn't be necessary when everything is dockerized.

### Step 1: Set up the watchtowers. _You can skip this step for the demo on Thursday_ {#demo-cslam-run-1}

To burn the SD card for each watchtower, the same instructions as for Duckiebots apply. [Duckiebot initialization](#setup-duckiebot)

The `hostname` of each watchtower should be of the form `demowatchtowerXX`, where `XX` stands for two digits (For example - demowatchtower01, demowatchtower02, and so on).

When the cards are burned, put in the watchtowers and their initialization is complete, install `roscore` and `ros-picam` if they are not there already:

    laptop $ docker -H ![hostname].local run -d --privileged --name roscore --network=host -v /data:/data --restart always duckietown/rpi-ros-kinetic-roscore:master18
    laptop $ docker -H ![hostname].local run -d --name ros-picam --network=host --device /dev/vchiq -v /data:/data --restart always duckietown/rpi-duckiebot-ros-picam:master18

It is also necessary to pull the docker image for the cSLAM acquistion node:

    laptop $ docker -H ![hostname].local pull duckietown/cslam-aquisition:rpi

### Step 2: Setup the AprilTags _You can skip this step for the Thursday demo_ {#demo-cslam-run-2}

Print out the AprilTags and place them on top of Duckiebots and in Duckietown. Configure a map description file for this. Check [duckietown-world](https://github.com/duckietown/duckietown-world) for an example. Aim to have neighboring towers seeing at least one common AprilTag and to have each watchtower see at least two AprilTags.

TODO: For Amaury: Explain where the config files for the city and the Duckiebots are by default and what needs to be chenged there by the user.

### Step 3: Setup a ROS Master machine {#demo-cslam-run-3}

One of the computers will act as ROS Master. That means that all other nodes and containers will need to have their `ROS_MASTER_URI` environment variable set up to this computer. To set this up, run `roscore` on a computer that has ROS Kinetic installed. Keep in mid that you mind need to source the ROS setup file before that with something like

    laptop $ source /opt/ros/kinetic/setup.bash

Then make a note of the hostname of the computer running the ROS Master. You will need that later. When you run `roscore` this will show in a line similar to:

```
auto-starting new master
process[master]: started with pid [3263]
ROS_MASTER_URI=http://![hostname]:11311/
```

You will also need the IP address of  this computer. The easiest way is to simply ping this hostname (followed by `.local`) from another computer.

### Step 4: Configure the watchtowers {#demo-cslam-run-4}

In order to start processing data on the watchtowers you need to run the `cslam-aquisition` container. This requires that you know the hostname and the IP address of the machine serving as the ROS Master for this demo. Once you know the hostname you can get its IP address by pinging it. You should have done this already in [Step 3](#demo-cslam-run-3).

We have made a `bash` script that allows to easily set up all the the watchtowers. You can find it in `duckietown-cslam/scripts/watchtowers_setup.sh`. You will need to edit the `ROS_MASTER_HOSTNAME` and `ROS_MASTER_IP` in this file to the ones of your ROS Master. Note that the `ROS_MASTER_HOSTNAME` should not contain `.local` at the end. Also check if `array` contains all your watchtowers. Then you can run it with `bash watchtowers_setup.sh`.

This step sets up the data acquisition pipeline on each watchtower. This means that each watchtower will now send updates about the AprilTags it sees. A similar step will also be done for each Duckiebot in [Step 6](#demo-cslam-run-6).

### Step 5: Test the watchtowers {#demo-cslam-run-5}

Setup the diagnostics tool to check that the status of the watchtowers are `OK` (AprilTag data was received in the last 5 seconds).

    laptop $ docker pull duckietown/cslam-diagnostics
    laptop $ docker run -it --rm --net=host --env="DISPLAY" -e ROS_MASTER_URI_DEVICE=![ROS_MASTER_HOSTNAME] -e ROS_MASTER_URI_DEVICE_IP=![ROS_MASTER_IP] duckietown/cslam-diagnostics

Note that `ROS_MASTER_HOSTNAME` should not contain `.local` at the end.

If some of the watchtowers does not appear in the list, then it was likely not configured properly. Sometimes this is due to connection issues. Try to repeat the previous step again.

You can also see the image stream from the watchtowers with `rqt_image_view`. On the computer running `roscore` run the following line:

    laptop $ rqt_image_view
    
Depending on your ROS configuration you might need to first source the ROS setup file with:

    laptop $ source /opt/ros/kinetic/setup.bash
    
You should now be able to see the raw and rectified watchtower streams, as well as a test stream that shows the detected AprilTags.

### Step 6: Setup the Duckiebot {#demo-cslam-run-6}

The Duckiebot should have the following 3 containers runnning:

- `roscore` [Instructions here](#rc-control)
- `ros_picam` (and `picam` should be stopped) [Instructions here](#read-camera-data)
- `joystick` [Instructions here](#rc-control)

You can start them or check if they are already running via Portainer.

As the Duckiebot usually has other nodes running we spare it processing of images and odometry by offloading this. To do this we need to run the acquisition node container on one of the computers. We recommend *not* using the computer serving as the ROS Master. The acquisition node has a lot of configuration parameters. That is where `docker-compose` is handy. You can check the example `.yml` file given: `duckietown-cslam/scripts/docker-compose-duckiebot-x86.yml`. Here you need to update a few lines:

- Replace `duckiebotHostname` in `acquisition_node_duckiebotHostname`, `ACQ_ROS_MASTER_URI_DEVICE=duckiebotHostname.local` and `ACQ_DEVICE_NAME=duckiebotHostname` with your Duckiebot's hostname.
- Replace `XXX.XXX.XXX.XXX` in `ACQ_ROS_MASTER_URI_DEVICE_IP` with your Duckiebot's IP address. You can get this if you ping it.
- Replace `ROS_MASTER_HOSTNAME` in `ACQ_ROS_MASTER_URI_SERVER=ROS_MASTER_Hostname.local` with your ROS Master's hostname. You should have gotten this already when you configured the watchtowers.
- Replace `XXX.XXX.XXX.XXX` in `ACQ_ROS_MASTER_URI_SERVER_IP` with your ROS Master's IP address. You should have gotten this already when you configured the watchtowers.

You can then start the container by running:

`docker-compose -f docker-compose-duckiebot-x86.yml up`

Make the Duckiebot see an AprilTag and you should see that you receive messages from it in the Diagnostics tool. You might have to restart the diagnostics tool to see the updates.

If you don't have docker-compose installed you can run the same command in the classical Docker way, althought it is much uglier:

     laptop $ docker run -it --name cslam-acquisition --restart always --network host 
     
           duckietown/cslam-acquisition:x86
      environment:
    -e ACQ_ROS_MASTER_URI_DEVICE=![duckiebotHostname].local -e ACQ_ROS_MASTER_URI_DEVICE_IP=![XXX.XXX.XXX.XXX] -e ACQ_ROS_MASTER_URI_SERVER=![ROS_MASTER_HOSTNAME].local -e ACQ_ROS_MASTER_URI_SERVER_IP=![XXX.XXX.XXX.XXX] -e ACQ_DEVICE_NAME=![duckiebotHostname]


TODO: This wasn't working live. I have verified that it works on bag files

### Step 7: Set up the cSLAM Graph Optimizer {#demo-cslam-run-7}

On a laptop that is connected to the same network as the rest:

TODO: Let's mention that we should do this on the server and not any laptop  

    laptop $ docker pull duckietown/cslam-server
    laptop $ docker run -it --rm --net=host -e ROS_MASTER_URI_DEVICE=![Rosmaster name] -e ROS_MASTER_URI_DEVICE_IP=![rosmaster IP] amaurx/cslam-graphoptimizer:latest /bin/bash

    container $ /graph_optimizer/catkin_ws/src/pose_graph_optimizer/wrapper.sh

This will listen to the transforms, will build a graph, optimize it and publish the output on TF, which you will visualize with Rviz in the next step.  

### Step 8: Set up the visualization {#demo-cslam-run-8}
Set up and run the visualization of the map, Duckiebots, watchtowers, and traffic signs using the following commands:

    laptop $ docker pull duckietown/cslam-visualization
    laptop $ docker run -it --rm --net=host --env="DISPLAY" -e ROS_MASTER_URI_DEVICE=[SERVER_HOSTNAME] -e ROS_MASTER_URI_DEVICE_IP=[SERVER_IP] duckietown/cslam-visualization

### Step 9: The fun part {#demo-cslam-run-9}
Control the Duckiebot manually around Duckietown

    laptop $ dts duckiebot keyboard_control ![duckie_hostname]

Look at the diagnostic tool to ensure the messaging status of the Duckiebots are `OK` where data was received in the last 10 seconds. If the Duckiebot messages does not appear in the list, then it was likely not configured properly. Sometimes this is due to connection issues.

### Step 10: Shut everything off {#demo-cslam-run-10}
You can stop the `cslam-acquisition` containers on the watchtowers with the `watchtowers_stop.sh` script in the `duckietown-cslam/scripts` folder. Before that check if all the watchtowers you are using are in the `array` in the script.

You can then stop the processing of your Duckiebot images and odometry by pressing <kbd>Ctrl</kbd>-<kbd>C</kbd> and executing:

`docker-compose -f docker-compose-duckiebot-x86.yml down`

TODO: If this has to be done for each Duckiebot, maybe we should have something like docker-compose-<duckiebot_hostname>-x86.yml

## Troubleshooting {#demo-cslam-troubleshooting}

### I can't connect to something.
Check if you are on the right network. Check if you can ping the device. Sometimes the network won't resolve hostnames and requires a restart.

### Rviz visualization makes no sense
If the positions of your Duckiebots and watchtower in Rviz make no sense, there is probably an issue among but not limited to the following:  
    - AprilTag recognition is off and gives out weird transforms  
    - Time delays between different input (watchtowers, Duckiebots) will lead to disconnected graphs that will not be useful. The whole idea is that the graph build and interpolates measures based on their time stamps. If differents actors are not synchronized or if one has delay, it will lead to bad results  
    - Optimization might take to long because of discrepencies in the graph. To get info on the optimization itself, check the argument `optim_verbose` in graph_builder.launch


### AprilTags printed may be of wrong size
Check that the printed AprilTags are of size 6.5cm as the printer might have done some scaling to the tags.

### Check Diagnostics tool
Check that messages are received frequently. Is not device may be suffering from poor connection and will need to restart or configuration for device was done wrongly and it needs to be reconfigured again. Do check that the network signal is strong enough for the devices to communicate with one another. Keep in mind that sometimes the Diagnostics tool won't show devices that connected after it started, so first try to restart it.

### How to see the g2o graph
In Rviz, you only see parts of the actual underlying g2o graph. If you want to visualize it, please have g2o_viewer installed.   
In graph_builder.launch, you can set the "save_g2o_output" argument to True for the optimization. This will create a text representation of the g2o graph in \tmp that you can visualize using  [g2o_viewer](#fig:g2o_viewer).   

<div figure-id="fig:g2o_viewer" figure-caption="g2o_viewer window">
     <img src="cSLAM_images/g2o_view.png" style='width: 30em'/>
</div>

## Demo failure demonstration {#demo-cslam-failure}

Finally, put here a video of how the demo can fail, when the assumptions are not respected.
