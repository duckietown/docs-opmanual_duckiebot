# Demo system ID {#demo-sysid status=deprecated}

This document provides an operation manual for performing an offline optimization of the odometry parameters. The odometry optimization is conducted in two steps. First, we execute a sequence of commands on the duckiebot and record its motion with the on-board camera. Then, we feed these information to an optimization script in our local computer to compute the odometry parameters. These parameters will be stored in the kinematics yaml file.

In order to complete the procedure, you need your duckiebot in configuration DB18 until unit B-11 with its camera calibrated. In addition, you need the checkerboard that you used during your camera calibration.


<div class='requirements' markdown="1">

Requires: Camera calibration, ROS installation on local computer

Requires: The duckiebot in configuration DB18 section B-11

Requires: Camera calibration completed.

</div>

## Video of expected results {#demo-sysid-expected}

<div figure-id="fig:demo_succeeded-sysid">
    <figcaption>Demo of the calibration procedure
    </figcaption>
    <dtvideo src='vimeo:251027149'/>
</div>


## Pre-flight checklist {#demo-sysid-pre-flight}

Check: the duckiebot has sufficient battery

Check: the USB drive is mounted

Check: the camera is calibrated

Check: ROS is installed on your local computer

Check: the checkerboard has the correct dimensions (31 mm squares)

Note: In case you experience dependencies errors latter while `roslaunch`ing to the calibration script, please make sure to install dependencies for your computer by running `dependencies_common.sh` and `dependencies_for_laptop.sh`. These executables are located under the Software repository. This software repository will be cloned by you in the step 7 below. Dependencies can be run in the Software repository by

    laptop $ ./dependencies_common.sh
    laptop $ ./dependencies_for_laptop.sh

## Demo instructions {#demo-sysid-run}

**Step 0**: Before starting to the experiment please install ROS on your local computer by following the official installation instructions [here](http://wiki.ros.org/kinetic/Installation/Ubuntu). Please install the Desktop-Full  version.

**Step 1**: Now we need to run the docker container to be able to launch the data collection script on our duckiebot. Check if DOCKER_HOST variable is set

    laptop $ echo $DOCKER_HOST

in case it is not, then set it

    laptop $ export DOCKER_HOST=![ROBOT_NAME].local

Now, run the docker container `sysid` on your duckiebot

    laptop $ docker -H ![ROBOT_NAME].local run -it --memory="800m"
    --memory-swap="1.8g" --net host --privileged -v /data:/data --name sysid duckietown/devel-sys-id:master18 /bin/bash

**Step 2**: Now we have to mount the USB storage to store the experiment data.

    duckiebot $ roscd calibration
    duckiebot $ sudo mkdir /media/logs
    duckiebot $ bash mount_usb

**Step 3**: Place your duckiebot in front of the checkerboard at a distance of slightly more than 1 meter in front of the checkerboard as shown in the image ([](#fig:calibration_setup)). __Please make sure that the checkerboard is oriented long side down as shown in the picture__.

You will probably observe that the duckiebot can't drive straight. You can adjust the initial heading of the duckiebot iteratively to maximize the time the duckiebot sees the checkerboard. Having enough data points is important to have a good fit.

<div figure-id="fig:calibration_setup" figure-caption="The calibration setup">
     <img src="calibration_setup.jpg" style='width: 30em'/>
</div>

**Step 4**: Run the calibration procedure

    duckiebot $ roslaunch calibration commands.launch veh:=![ROBOT_NAME] vFin:=![vFin] Nstep:=![Nstep] k1:=![k1] k2:=![k2] omega:=![omega] duration:=![duration]

Note that you can configure ramp and sine inputs. We have two parameters that we can configure for the ramp input:

`vFin` is final voltage value for the ramp up experiment with default of 0.5 <br/>
`Nstep` is step size for the ramp up experiment with default of 180 <br/>

To decrease the duration of the ramp input you could decrease the value of `Nstep`. Also if you would like to decrease the final speed of your duckiebot during the ramp experiment you can set `vFin` to a lower value.

We have four parameters that we can configure for the sine input:

`K1` denotes the mean command for the sine calibration with default of 0.2, <br/>
`K2` denotes the amplitude of the sine curve with default of 0.06, <br/>
`omega` is angular velocity with default of 0.007, <br/>
`duration` denotes duration for the sine calibration with default of 2000 ms.<br/>

After executing the roslauch command the duckiebot should go forward and then stop.

**Step 5**: When the duckiebot stopped, you have 10 seconds to replace it again at a distance of approximately 1 meters from the chessboard. Wait for the duckiebot to move forward again.

When the duckiebot stops and the node shuts down we can proceed to the actual optimization stage. Now you should unmount the USB drive from the duckiebot using the following command.

    duckiebot $ sudo umount /media/logs

Now, plug out the USB from your duckiebot and plug it in your laptop to copy the .bag file.

**Step 6**: Get the files from the calibration folder (required) of your duckiebot to your laptop using

        laptop $ scp -r ![ROBOT_NAME]:/data/config/calibrations/ ~/duckietown_sysid

It will create a folder named duckietown_sysid in your home directory, where all the calibration files will be stored. Please make sure that you do copy the calibrations folder to the local `~/duckietown_sysid`

**Step 7**: On your computer, create a duckietown folder and clone the git repository [duckietown/Software](https://github.com/duckietown/Software) in the duckietown folder using the following commands:

    laptop $ cd ~
    laptop $ mkdir duckietown
    laptop $ cd duckietown
    if by SSH laptop $ git clone git@github.com:duckietown/Software.git
    if by HTTP laptop $ git clone https://github.com/duckietown/Software.git

Check that you have a folder `Software` in the duckietown folder. If you execute `cd Software`, you will be on master branch by default. For this tutorial you have to `checkout devel-sysid-tutorial` to be in the right branch.

Note that at this point you must have the ROS installed on your local computer

Activate ROS:

    laptop $ cd ~/duckietown/Software/catkin_ws
    laptop $ catkin_make
    laptop $ cd ..
    laptop $ source environment.sh

This catkin_make is executed in the catkin_ws folder on the `devel-sysid-tutorial` git branch.


**Step 9**: Run the calibration process with

    laptop $ roslaunch calibration calibration.launch veh:=ROBOT_NAME  path:=/absolute/path/into/the/rosbag/folder/

An path example would be `path:=/home/user_name/sysid/`. Pay attention to the the backslash at the end of the path and to start your path from `/home`.

At the end you will receive two plots showing the open loop predictions of the duckiebot's x position, y position and yaw angle using the initial parameters guesses (marked as def on the plots) and the guesses with the optimized parameters (marked as opt). The actual measurements will also be shown on the plots for assessing the performance of both models.

**Step 10**: Once the command has finished, the parameters of your duckiebot are stored in the folder `~/duckietown_sysid/kinematics/![ROBOT_NAME].yaml`

**Step 11**: Transfer the generated file with optimized parameters into the duckiebot, using the following set of commands:

Please verify that the trim and gain values that you see in your terminal is written to the kinematic calibration file under `~/duckietown_sysid/kinematics/![ROBOT_NAME].yaml`. To transfer the kinematic calibration file back to your duckiebot execute

    duckiebot $ scp -r ![LOCAL_PC_USERNAME]@![LOCAL_PC_NAME]:~/duckietown_sysid/kinematics /data

To replace the old kinematic file with the new one, on your duckiebot type:

    duckiebot $ sudo rm -rf /data/config/calibrations/kinematics
    duckiebot $ sudo mv /data/kinematics /data/config/calibrations

**Step 12** (Optional): Run the Validation: The duckiebot should first drive straight for 1m in 5s, then turn a full circle to the left and then a full circle to the right.

    duckiebot $ roslaunch calibration test.launch veh:=![ROBOT_NAME]

known issue: the baseline is rather overestimated at the moment, thus your duckiebot will probably turn more than a circle


## Troubleshooting {#demo-sysid-troubleshooting}

Symptom: No log have been recorded.

Resolution: Try to mount the USB drive.

Symptom: The duckiebot deviates from the trajectory, so that the chessboard goes out of the camera’s field of view.

Resolution: You can adjust the parameters of the voltage commands by passing arguments when launching the commands. You can change the parameter vFin and Nstep for the straight line, and the parameter k1, k2, omega and duration for the sinewave.

Symptom: Issues/bugs with copying from USB to computer. USB cannot be unmounted from the duckiebot.
Remounting USB is not possible without rebooting the duck
After the first sftp get the USB drive becomes „read only“ and no further bags can be recorded

Symptom: There are large discontinuities in the recordings despite the fact that the duckiebot does see the checkerboard most of the time.   

Resolution: One possible cause of this problem is insufficient memory. Please make sure to execute `docker run` command with `--memory="800m" --memory-swap="1.8g"` flags which would tell docker to utilize the swap space. Swap space is created and allocated during the initialization process. The swap space allocation is done by default since 5 October 2018. If you had flashed your SD card prior to that, please reflash your SD card. You can verify that you have swap space by executing `top` command in your duckiebot and inspecting `KiB Swap` section.


## Demo failure demonstration {#demo-sysid-failure}

<div figure-id="fig:demo_failed-sysid">
    <figcaption>Failed demo
    </figcaption>
    <dtvideo src='vimeo:251027122'/>
</div>
