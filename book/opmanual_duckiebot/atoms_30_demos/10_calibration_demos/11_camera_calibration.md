# Camera Calibration Verification Test {#demo-camcaltest status=beta}

This document provides instructions for testing the camera calibration.

<div class='requirements' markdown="1">

Requires: A duckiebot version DB-18.

Requires: USB drive.

Requires: Camera calibration verification hardware.

Results: A validation of the current camera calibration.

</div>


## Pre-flight checklist {#demo-camcaltest-pre-flight}

Check: the USB plugged in

Check: the duckiebot has sufficient battery

Check: you have camera calibration verification hardware ready

## Abbreviations {#demo-camcaltest-abb}

`DOCKER_CONTAINER` = duckietown/rpi-duckiebot-base:suitability-suite-v1

`SOFTWARE_ROOT` = /home/software

`PACKAGE_ROOT` = /catkin_ws/src/05_teleop/calibration

## Demo instructions {#demo-camcaltest-run}


**Step 1**: SSH into your duckiebot and create the directory for logging.

    duckiebot $ sudo mkdir /data/logs

**Step 2**: Then mount your USB

    duckiebot $ sudo mount -t vfat /dev/sda1 /data/logs -o umask=000

**Step 3**: Now we will run the docker container on our duckiebot that contains the test script. Open a new terminal on your computer. Make sure that `DOCKER_HOST` variable is set by checking

    laptop $ echo $DOCKER_HOST

if the output is empty then `DOCKER_HOST` is not set. You can set it with

    laptop $ export DOCKER_HOST=![ROBOT_NAME].local

Now, we will run the docker container. Be sure to replace the `DOCKER_CONTAINER` with the name provided under [Abbreviations section](#demo-camcaltest-abb).

    laptop $ docker -H ![HOST_NAME].local run -it --net host --privileged -v /data/logs:/logs -v /data:/data --memory="800m" --memory-swap="2.8g" --name suitability-suite ![DOCKER_CONTAINER] /bin/bash

Depending on you network speed it might take some time until the duckiebot downloads the container.

**Step 4**: Now Place your duckiebot inside the camera calibration hardware.

Note: If you just executed *Step 3*, a shell inside the container should be already open.

Having the experimental setup ready, we can start testing, enter into the running `camera-test ` container if your are not already in, and launch the test interface.

    laptop $ export DOCKER_HOST=![ROBOT_NAME].local

    laptop $ docker exec -it suitability-suite /bin/bash

And then launch the camera calibration.

    duckiebot-container $ roslaunch calibration camera_calibration_test.launch veh:=![HOST_NAME] output_dir:=/logs

Note that, if `output_dir` is not specified the program attempts to save the results to user´s home folder. This will fail it you don't have enough space in your device.

With data-acquisition interface you can specify

* the type of the experiment you would like to conduct by choosing amongst the presented options,

* whether to save the collected experiment data by replying to the question after the experiment has been completed,

* whether to do another experiment.


The results of the experiment can be found under `/logs` folder in a zipped form. To download the results ![ZIPPED_RESULT_NAME] to your local computer first move the zipped folder to your `/data` folder,

    duckiebot-container $ mv /logs/![ZIPPED_RESULT_NAME] /data/

Now you can download the file by heading to `![HOST_NAME]:8082` in your **browser** and then clicking on `![ZIPPED_RESULT_NAME]`.  

## Troubleshooting {#demo-camcaltest-troubleshooting}

Symptom: No log have been recorded.

Resolution: Make sure you mounted USB drive. Please note that you have to should first mount it correctly before you can start data collection.

Symptom: Logs are created but they are empty.

Resolution: This might be because the Raspberry-Pi did not have enough time to save the data. Please increase `wait_start_rosbag` and `wait_write_rosbag` inside [this script](https://github.com/selcukercan/Software/blob/system-identificiation-v1/catkin_ws/src/05-teleop/calibration/src/data_collector.py).

Symptom: While shutting-down program complains,`ROSException: publish() to a closed topic`.

Resolution: This is a known problem and it does not interfere with the healthy operation of the program. It is in the to-do list to have a graceful execution termination. Feel free to contribute with a pull request if you resolve this issue.
