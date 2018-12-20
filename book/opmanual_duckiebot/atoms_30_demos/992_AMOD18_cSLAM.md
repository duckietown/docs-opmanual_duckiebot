# AMOD18 cSLAM {#demo-cslam status=beta}

<div figure-id="fig:cslam_logo">
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

Results: Global locations of Duckiebots and other objects with AprilTags in a Duckeitown city

Results: A 3D map of the city that shows the moving Duckiebots

</div>

## Video of expected results {#demo-cslam-expected}

First, we show a video of the expected behavior (if the demo is successful).

## Demo code {#demo-code}

The main cSLAM repository is [here](https://github.com/duckietown/duckietown-cslam). When we refer to configuration files or scripts, they will be here. Clone it for easy access.

There are also more detailed about the code, different configuration parameters and how to maintain it.

## Introduction to cSLAM {#demo-introduction}

It can be incredibly useful to have a system that can localize your Duckeibot. Not only in case you lose it, but also if you want to facilitate autonomous Robotarium operations or to evaluate AIDO submissions. And most importantly, it looks cool! And that's exactly what the cSLAM system does.

The goal of cSLAM is to fuse the observations of watchtowers and Duckiebots in a single optimization that tries to predict as accurately as possible the current and past Duckiebot locations. This is then nicely visualized on a 3D model of the city.

cSLAM was designed such that it is modular, scalable, and with minimum overhead on the Duckiebots. It is a complex system, so modular means that you will need to run quite a few Docker containers. On the other hand, it is easy to support and extend. And don't worry, we've automated the bulk of container work. Scalable means that it can easily be extended to large cities. As the system is modular, the processing can be distributed over more computers as a city grows. Finally, the minimum overhead means that we don't run any special code on the Duckiebots, we only expect them to publish image and odometry data. All the processing is offloaded to a computer.

At the core of cSLAM is a pose graph optimization problem. The watchtowers observe AprilTags on the ground, on top of Duckiebots and even on traffic signs. They can then estimate the relative pose of these AprilTags. Duckiebots similarly see tags on the ground and on traffic signs and estimate relative poses. Duckiebots also estimate their own pose relative to their past position by using odometry data. All these poses, together with the times when they were observed, are combined in a graph optimization problem that is solved by a library called [g2o](https://github.com/RainerKuemmerle/g2o). The solution to this problem is the global positions of all the observed AprilTags, and consequently of the Duckiebots.

<div figure-id="fig:architecture" figure-caption="Architecture of cSLAM.">
     <img src="cSLAM_images/architecture.png" style='width: 40em'/>
</div>

As can be seen from the above image there are quite a few devices and containers involved in this demo. Every red box is a physical device; every blue box is a Docker container that can be deployed on a different device. Let's see what the different containers are and what they do:

* The *acquisition containers* are multiple Docker containers, each being responsible for acquiring the raw data from a single robot (watchtower or Duckiebot), processing it (rectifying, April tag pose extraction, odometry calculation), and packaging the processed data as new topics on the Graph optimization ROS Master.

    Using separate containers allow us to scale the system to an almost infinite amount of robots and watchtowers (simply run as many nodes as necessary on a docker swarm). In this demo, we will run the acquisition containers for watchtowers on the towers themselves as this reduces the network delays in the system. The acquisition containers for Duckiebots, however, will be executed on a laptop such that the Duckiebot's computational resources are available for other processes.

* The *Graph optimizer* container aggregates all the AprilTag and odometry information, builds a pose graph out of it, and optimizes this graph. Then it publishes global pose information for all AprilTags and cameras in the system, which includes the positions of Duckiebots, watchtowers, traffic signs, and ground tags.

* The *Visualization* container presents the results of the graph optimization in a human- (and duckie-) friendly interface. It reads the positions of the various objects from the Graph optimizer container and shows them on a 3D map of the city.

* The *Diagnostics* container constantly probes the AprilTag pose and odometry messages that are being published and signals if a device stops publishing. This is useful to detect network or configuration issues.


## Duckietown setup notes {#demo-cslam-duckietown-setup}
We have the following basic assumptions that need to be fulfilled in order for the demo to work.

### Layout
  * Traffic lights are good to have but not necessary (optional).
  * Each floor AprilTag should have at least two watchtowers seeing it. It is recommended, but not necessary, that the bipartite graph of watchtowers and AprilTags are connected.
### Infrastructure
  * AprilTags have to be placed on the tiles, traffic signs, and Duckiebots. These tags should all be unique!
  * For a detailed map to be visualized, the poses of the AprilTags in the Duckietown must be (approximately) known beforehand.
  * Watchtowers have to be spread across the entire Duckietown. Preferably the combined field of view covers the entire Duckietown.
### Weather
  * Lighting has to be bright enough for the AprilTags to be seen clearly by the cameras. Still not too bright to have unwanted reflections.
  * Assumption: it is always sunny. Rain never occurs in Duckietown.

## Duckiebot setup notes {#demo-cslam-duckiebot-setup}

* All Duckiebots are required to have an AprilTag mounted on top of them.
* The bottom side of the AprilTag has to be pointing towards the rear of the Duckiebot.
* The center of the AprilTag should be aligned as closely as possible to the center of gravity of the Duckiebot.


## Pre-flight checklist {#demo-cslam-pre-flight}

<div class='check' markdown="1">

* The Duckiebot has sufficient battery and is powered on.

* The `roscore`, `ros-picam`, `joystick`, and `keyboard_control` containers are turned on for each Duckiebot.

* The watchtowers are powered on

* ROS is installed on your local computer. If it is not, install ROS Kinetic on your local computer by following the official installation instructions [here](http://wiki.ros.org/kinetic/Installation/Ubuntu). Please install the Desktop-Full version.

* Docker is installed on your local computer. If it is not, install Docker on your local computer by following the official installation instructions [here](https://docs.docker.com/install/linux/docker-ce/ubuntu/). It is recommended that you also have [Docker-compose](https://docs.docker.com/compose/install/)

</div>


## Demo instructions {#demo-cslam-run}

### Step 0: Preliminaries {#demo-cslam-run-0}

* Clone duckietown-cslam : https://github.com/duckietown/duckietown-cslam

* Make sure all devices you are using are connected to the same WiFi network (typically that would be `duckietown`)

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

TODO: For Amaury: Explain where the config files for the city and the Duckiebots are by default and what needs to be changed there by the user.

### Step 3: Setup a ROS Master machine {#demo-cslam-run-3}

One of the computers will act as ROS Master. That means that all other nodes and containers will need to have their `ROS_MASTER_URI` environment variable set up to this computer. To set this up, run `roscore` on a computer that has ROS Kinetic installed. Keep in mind that you might need to source the ROS setup file before that with the following command:  

    laptop $ source /opt/ros/kinetic/setup.bash

Then run,

    laptop $ roscore

Make a note of the hostname of the computer running the ROS Master. You will need that later. The hostname can also be seen after running `roscore`, where it will show it in a line similar to:

```
auto-starting new master
process[master]: started with pid [3263]
ROS_MASTER_URI=http://![hostname]:11311/
```

You will also need the IP address of this computer. The easiest way is to simply ping this hostname (followed by `.local`) from another computer. You can also get the IP address by running `ifconfig` on this computer (make sure you look at the correct network interface).

### Step 4: Configure the watchtowers {#demo-cslam-run-4}

In order to start processing data on the watchtowers, you need to run the `cslam-aquisition` container. You already know the hostname and the IP address of the machine serving as the ROS Master for this demo from [Step 3](#demo-cslam-run-3).

We have made a `bash` script that allows to easily set up all the watchtowers. You can find it in `duckietown-cslam/scripts/watchtowers_setup.sh`. You will need to edit the `ROS_MASTER_HOSTNAME` and `ROS_MASTER_IP` in this file to the ones of your ROS Master. Note that the `ROS_MASTER_HOSTNAME` should not contain `.local` at the end. Also check if `array` contains all your watchtowers (Once again, you can ignore this for the demo on Thursday.) Then, go to directory `duckietown-cslam/scripts/` and run it:

     laptop $ ./watchtowers_setup.sh

This step sets up the data acquisition pipeline on each watchtower. This means that each watchtower will now send updates about the AprilTags it sees. A similar step will also be done for each Duckiebot in [Step 6](#demo-cslam-run-6).

### Step 5: Test the watchtowers {#demo-cslam-run-5}

Setup the Diagnostics tool to check that the status of the watchtowers are `OK` (AprilTag data was received in the last 5 seconds).

    laptop $ docker pull duckietown/cslam-diagnostics
    laptop $ xhost +local:root
    laptop $ docker run -it --rm --net=host --env="DISPLAY" -e ROS_MASTER=![ROS_MASTER_HOSTNAME] -e ROS_MASTER_IP=![ROS_MASTER_IP] duckietown/cslam-diagnostics

Note that `ROS_MASTER_HOSTNAME` should not contain `.local` at the end.

NOTE: After everything is over, please run:  

    laptop $ xhost -local:host

If some of the watchtowers does not appear in the list, then it was likely not configured properly. Sometimes this is due to connection issues. Try to repeat the previous step again.

You can also see the image stream from the watchtowers with `rqt_image_view`. On the computer running `roscore` run the following line:

    laptop $ rqt_image_view

Depending on your ROS configuration you might need to first source the ROS setup file with:

    laptop $ source /opt/ros/kinetic/setup.bash

You should now be able to see the raw and rectified watchtower streams, as well as a test stream that shows the detected AprilTags.

### Step 6: Setup the Duckiebots {#demo-cslam-run-6}

The Duckiebots should have the following 3 containers runnning:

- `roscore` [Instructions here](#rc-control)
- `ros_picam` (and `picam` should be stopped) [Instructions here](#read-camera-data)
- `joystick` [Instructions here](#rc-control)

You can start them or check if they are already running via Portainer.

As the Duckiebots usually have other nodes running we spare them processing of images and odometry by offloading this. To do this we need to run an acquisition node container for each one of the Duckiebots any of the computers on the same network. We recommend *not* using the computer serving as the ROS Master. The acquisition node has a lot of configuration parameters. That is where `docker-compose` is handy. You can check the given example file: `duckietown-cslam/scripts/docker-compose-duckiebot-x86.yml`. You can copy the lines from `acquisition_node_duckiebotHostname` as many times as Duckiebots you have. Here you need to update a few lines for each one of these:

- Replace `duckiebotHostname` in `acquisition_node_duckiebotHostname`, `ACQ_ROS_MASTER_URI_DEVICE=duckiebotHostname.local` and `ACQ_DEVICE_NAME=duckiebotHostname` with your Duckiebot's hostname.
- Replace `XXX.XXX.XXX.XXX` in `ACQ_ROS_MASTER_URI_DEVICE_IP` with your Duckiebot's IP address. You can get this if you ping it.
- Replace `ROS_MASTER_HOSTNAME` in `ACQ_ROS_MASTER_URI_SERVER=ROS_MASTER_Hostname.local` with your ROS Master's hostname. You should have gotten this already when you configured the watchtowers.
- Replace `XXX.XXX.XXX.XXX` in `ACQ_ROS_MASTER_URI_SERVER_IP` with your ROS Master's IP address. You should have gotten this already when you configured the watchtowers.

You can then start all the containers simultaneously by running:

    laptop $ docker-compose -f docker-compose-duckiebot-x86.yml up

Make the Duckiebots see an AprilTag and you should see that you receive messages from them in the Diagnostics tool. You might have to restart the Diagnostics tool to see the updates.

If you don't have docker-compose installed you can run the same commands in the classical Docker way, although it is much uglier. You need to run the following command for every Duckiebot you use.

     laptop $ docker run -it --name cslam-acquisition --restart always --network host -e ACQ_ROS_MASTER_URI_DEVICE=![duckiebotHostname].local -e ACQ_ROS_MASTER_URI_DEVICE_IP=![XXX.XXX.XXX.XXX] -e ACQ_ROS_MASTER_URI_SERVER=![ROS_MASTER_HOSTNAME].local -e ACQ_ROS_MASTER_URI_SERVER_IP=![XXX.XXX.XXX.XXX] -e ACQ_DEVICE_NAME=![duckiebotHostname] duckietown/cslam-acquisition:x86

### Step 7: Set up the cSLAM Graph Optimizer {#demo-cslam-run-7}

In folder scripts, there is a `yaml` file called `apriltagsDB_custom.yaml`. In this file should be added all Duckiebots with their AprilTag number following the already existing convention. This file will be mounted and read inside the container. If it is not filled, your Duckiebots will not be seen as such.

On the ROS master machine defined at step 3, run:  

    laptop $ docker pull duckietown/cslam-graphoptimizer
    laptop $ docker run -it --rm --net=host -v $(pwd)/scripts/apriltagsDB_custom.yaml:/graph_optimizer/catkin_ws/src/pose_graph_builder/data/apriltagsDB_custom.yaml -e ROS_MASTER=![ROS_MASTER_HOSTNAME] -e ROS_MASTER_IP=![ROS_MASTER_IP] duckietown/cslam-graphoptimizer:latest /bin/bash

    laptop-container $ /graph_optimizer/catkin_ws/src/pose_graph_optimizer/wrapper.sh

You can also remove the `/bin/bash` and the wrapper will be executed directly. Keeping the `/bin/bash` is helpful for troubleshooting when you want to modify launch parameters.

This will listen to the transforms, will build a graph, optimize it and publish the output on TF, which you will visualize with Rviz in the next step.  

### Step 8: Set up the visualization {#demo-cslam-run-8}
Set up and run the visualization of the map, Duckiebots, watchtowers, and traffic signs using the following commands:

    laptop $ docker pull duckietown/cslam-visualization
    laptop $ xhost +local:root
    laptop $ docker run -it --rm --net=host --env="DISPLAY" -e ROS_MASTER=![ROS_MASTER_HOSTNAME] -e ROS_MASTER_IP=![ROS_MASTER_IP] duckietown/cslam-visualization

NOTE: After everything is over, please run:  

    laptop $ xhost -local:host

### Step 9: The fun part {#demo-cslam-run-9}

If you managed to get all the way to here, congratulations! Quack, quack, hooray!

Now you can drive a Duckiebot around the city and see how it moves on the map. To control the Duckiebot manually around city use the keyboard control:

    laptop $ dts duckiebot keyboard_control ![duckiebot_hostname]

Look at the Diagnostics tool to ensure the messaging status of the Duckiebots are `OK` where data was received in the last 5 seconds. If the Duckiebot messages do not appear in the list, then it was likely not configured properly. Sometimes this is due to connection issues.

### Step 10: Shut everything off {#demo-cslam-run-10}
You can stop the `cslam-acquisition` containers on the watchtowers with the `watchtowers_stop.sh` script in the `duckietown-cslam/scripts` folder. Before that check if all the watchtowers you are using are in the `array` in the script.

You can then stop the processing of Duckiebot images and odometry by pressing <kbd>Ctrl</kbd>-<kbd>C</kbd> in the terminal running docker-compose and by then running:

    laptop $ docker-compose -f docker-compose-duckiebot-x86.yml down

Make sure to execute the following command on your laptop! Otherwise, you expose it to security issues!

    laptop $ xhost -local:host

## Troubleshooting {#demo-cslam-troubleshooting}

Symptom: The positions of your Duckiebots and watchtowers in Rviz make no sense.

Resolution: There is probably an issue among but not limited to the following:

  * AprilTag recognition is wrong and gives out weird transforms. If this happens, please check that the printed AprilTags are of size 6.5cm, as the printer might have done some scaling to the tags.
  * There are time delays between different inputs (watchtowers, duckiebots). These lead to a disconnected pose graph: the graph builds and interpolates measures based on their timestamps, so if different actors are not synchronized or if one has a delay, bad results will be produced. Please check the frequency with which messages are received by using the [Diagnostics tool](#demo-cslam-run-5).
  * Optimization takes too long because of discrepancies in the graph. To get info on the optimization itself, check the argument `optim_verbose` in `graph_builder.launch`.
  Furthermore, you can visualize the underlying g2o graph. To do so, please have g2o_viewer installed. Then, in `graph_builder.launch` please set the `save_g2o_output` argument to True: this will create a text representation of the g2o graph in `\tmp` that you can visualize using [g2o_viewer](#fig:g2o_viewer).   
<div figure-id="fig:g2o_viewer" figure-caption="Snapshot of a g2o_viewer window.">
     <img src="cSLAM_images/g2o_view.png" style='width: 35em'/>
</div>

Symptom: Docker complains about wrong Docker versions

Resolution: If you run into this issue when running `watchtowers_setup.sh` or `watchtowers_stop.sh`, please run the script again. This typically resolves the error.

Symptom: Docker cannot connect to a remote device

Resolution: Try to run the command again. Often it is a temporary issue. If it persists, make sure you are on the correct network, that the device is powered on and that you can ping it.

Symptom: The Diagnostics tool doesn't show a device or shows ERROR

Resolution: This could be a non-problem. For example, if a watchtower or a Duckiebot doesn't recognize any AprilTags you will not receive messages. You can check that with the test image stream from this device in `rqt_image_view`. If the test image stream shows ids of AprilTags on the image, all you need to do is restart the Diagnostics tool.

If the problem persists, use Portainer to check if the `roscore`, `ros-picam`, and `cslam-acquisition` containers are running for this device. Check the logs of `cslam-acquisition` for errors.

Symptom: The Diagnostics tool shows that reception of messages is delayed (e.g. by 10-20 secs).

Resolution: The device may be suffering from a poor connection. Please make sure that the network signal is strong enough for the devices to communicate with one another and restart the devices. Keep in mind that sometimes the Diagnostics tool won't show devices that connected after it started, so if this is the case, try to restart it.

Symptom: Some messages that you expected to see (e.g. from a certain watchtower) do not show up in the Diagnostics tool.

Resolution: The configuration for the device was done wrong. Please check that the variables set by the user (e.g. `ROS_MASTER_URI_DEVICE`, `ROS_MASTER_URI_DEVICE_IP`, `ACQ_ROS_MASTER_URI_DEVICE`, `ACQ_ROS_MASTER_URI_DEVICE_IP`, etc.) are set coherently with the instructions above. If you find some inconsistencies, please follow the configuration steps again.

## Demo failure demonstration {#demo-cslam-failure}

Finally, put here a video of how the demo can fail, when the assumptions are not respected.
