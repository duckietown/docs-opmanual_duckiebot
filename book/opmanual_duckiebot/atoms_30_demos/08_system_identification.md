# Demo system ID {#demo-sysid status=beta}

This is the description of the odometry optimisation procedure. In order to complete the procedure, you need your Duckiebot in configuration DB18 untill unit B-11 with its wheel and camera calibrated, also you need the same checkerboard as for the camera calibration.

In the first step, you will put your Duckiebot in front of the checkerboard and send specific commands to the wheels. Using the checkerboard, the duckiebot will be able to localize itself. On your computer, you will then use these informations to optimize the the calibration parameters of your Duckiebot. These parameters will be stored in a yaml file.

Requires: Camera and wheel calibration completed

<div class='requirements' markdown="1">

Requires: Duckiebot in configuration DB18 section B-11

Requires: Camera calibration completed. Better to have wheel's calibrated, but not necessary.

</div>

## Video of expected results {#demo-sysid-expected}

<div figure-id="fig:demo_succeeded-sysid">
    <figcaption>Demo of the calibration procedure
    </figcaption>
    <dtvideo src='vimeo:251027149'/>
</div>



## Duckietown setup notes {#demo-sysid-duckietown-setup}

The Duckietown is not needed for the wheels calibration.


## Duckiebot setup notes {#demo-sysid-duckiebot-setup}

Mount the USB drive.

See: The procedure is documented in [](+software_reference#mounting-usb).


## Pre-flight checklist {#demo-sysid-pre-flight}

Check: the Duckiebot has sufficient battery

Check: the USB drive is mounted

Check: the camera is calibrated

Check: the checkerboard has the correct dimensions (31 mm squares)


## Demo instructions {#demo-sysid-run}

Check if DOCKER_HOST variable is set by using

    echo $DOCKER_HOST
if not, then set it using

```shell
export DOCKER_HOST=hostname.local
```
Now, run the docker container `sysid` on your bot using the following commands

```shell
docker -H hostname.local run -it --net host --privileged --name sysid duckietown/devel-sys-id:master18 /bin/bash
```
Step 2: Mount the USB Storage: To do this, you can use procedure is documented in [](+software_reference#mounting-usb) in the duckiebook or run the following commands on your duckiebot.

    duckiebot $ roscd calibration
    duckiebot $ sudo mkdir /media/logs
    duckiebot $ bash mount_usb

Step 3: Place the Duckiebot in front of the checkerboard at a distance of slightly more than 1 meter in front of the checkerboard as shown in the image ([](#fig:calibration_setup)).
The heading has to be set iteratively to maximize the time the duckiebot sees the checkerboard.

<div figure-id="fig:calibration_setup" figure-caption="The calibration setup">
     <img src="calibration_setup.jpg" style='width: 30em'/>
</div>

Step 4: Run the calibration procedure

    duckiebot $ roslaunch calibration commands.launch veh:=![robot name] vFin:=![vFin] Nstep:=![Nstep] k1:=![k1] k2:=![k2] omega:=![omega] duration:=![duration]

Here, vFin is final command value for straight calibration. Default for vFin is 0.5 <br/>
      Nstep is step size for the straight calibration. Default for Nstep is 180 <br/>
      K1 denotes the mean command for the sine calibration. Default for K1 is 0.2 <br/>
      K2 denotes the amplitude of the sine curve. Default for K2 is 0.06 <br/>
      omega is angular velocity. Default for omega is 0.007 <br/>
      Duration denotes duration for the sine calibration. Default for duration is 2000 <br/>

The Duckietown should go forward and then stop.

Step 5 When the Duckiebot has stopped, you have 10 seconds to replace it again at a distance of approximately 1 meters of the chessboard. Wait for the Duckiebot to move forward again.

When the Duckiebot stops, and the node shuts down, you have 2 different alternatives to copy the rosbag to the computer. (6a or 6b)

Step 6a:
    duckiebot $ sudo umount /media/logs

And put the USB drive in your computer.

Step 6b: cd to folder where you want to put the rosbag

    laptop $ sftp robot_name
    laptop $ cd /media/logs
    laptop $ get robot_name_calibration.bag

Step 7: Get the files from the calibration folder (required) of the duckiebot to your laptop using scp

       scp -r ![HOST_NAME]:/data/config/calibrations/\{kinematics,camera_extrinsic,camera_intrinsic\} ~/duckietown_sysid

It will create a folder named duckietown_sysid in your home directory, where all the calibration files will be stored.

Step 8: On your computer, create a duckietown folder and clone the git repository duckietown/Software in the duckietown folder using the following commands:

    laptop $ mkdir duckietown
    laptop $ cd duckietown
    if by SSH laptop $ git clone git@github.com:duckietown/Software.git
    if by HTTP laptop $ git clone https://github.com/duckietown/Software.git

Check that you will have a folder `Software` in the duckietown folder. If you enter the `cd Software`, you will be on master branch by default. For this tutorial you have to go to the `devel-sysid-tutorial` branch.

Activate ROS:

    laptop $ source environment.sh
    laptop $ cd catkin_ws
    laptop $ catkin_make

This catkin_make is done in the catkin_ws folder on the `devel-sysid-tutorial` git branch.


Step 9: Run the calibration process with

    laptop $ roslaunch calibration calibration.launch veh:=robot name  path:=/absolute/path/into/the/rosbag/folder/

(path example: path:=/home/user_name/sysid/) Do not forget the backslash at the end of the path.(Common mistake: path not starting from /home, forgetting the last / in the path)

Step 10: Once the command has finished, the parameters of your Duckiebot are stored in the folder

    ~/duckietown_sysid/kinematics/![robot name].yaml

Step 11: Load the generated file with optimised parameters into the duckiebot, using the following set of commands:
Note that it is just delete, copy and paste of the parameters file, you can use any other way. One of the way is shown by the code below.

    scp -r ~/duckietown_sysid/kinematics hostname:/data

On the duckiebot type:

    sudo rm -rf /data/config/calibrations/kinematics
    sudo mv /data/kinematics config/calibrations

Step 12 (Optional): Run the Validation: Duckiebot should first drive straight for 1m (in 5s) then turn a full circle to the left (in 8s) and then a full circle to the right (in 8s)

    duckiebot $ roslaunch calibration test.launch veh:=hostname

known issue: the baseline is rather overestimated at the moment, thus the duckiebot will probably turn more than a circle


## Troubleshooting {#demo-sysid-troubleshooting}

Symptom: No log have been recorded.

Resolution: Try to mount the USB drive.

Symptom: The Duckiebot deviates from the trajectory, so that the chessboard goes out of the camera’s field of view.

Resolution: You can adjust the parameters of the voltage commands by passing arguments when launching the commands. You can change the parameter vFin and Nstep for the straight line, and the parameter k1, k2, omega and duration for the sinewave.

Sympton: Issues/bugs with copying from USB to computer. USB cannot be unmounted from the duckiebot (mine at least)
Remounting USB is not possible without rebooting the duck
After the first sftp get the USB drive becomes „read only“ and no further bags can be recorded

## Demo failure demonstration {#demo-sysid-failure}

<div figure-id="fig:demo_failed-sysid">
    <figcaption>Failed demo
    </figcaption>
    <dtvideo src='vimeo:251027122'/>
</div>
