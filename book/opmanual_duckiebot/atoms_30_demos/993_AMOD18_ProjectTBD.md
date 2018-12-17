# AMOD18 Project TBD {#demo-projecttbd status=draft}

This is the demo for object detection using the camera on the duckiebot. The duckiebot has been trained to detect duckies, duckiebots, traffic lights, QR codes, intersection signs, stop signs, and (traffic) signal signs. The joystick demo (or lane following pipeline) is used to navigate the duckiebot around duckietown.


<div class='requirements' markdown="1">

Requires: Two or more duckiebots initialisation completed.

Requires: Camera calibration completed and wheel calibration completed.

Requires: Joystick demo completed.

Requires: Duckietown with all classified objects.
</div>

## Video of expected results {#demo-projecttbd-expected}

First, we show a video of the expected behavior (if the demo is succesful).

## Duckietown setup notes {#demo-projecttbd-duckietown-setup}

The Duckietown used for this demo must have the following characteristics.
* Several duckies placed on the road and at the side of the road.
* Intersection sign, Stop sign and Signal sign.
* QR codes on the ground and below the signs mentioned above.
* Traffic lights at intersections
* No cluttering of objects in one place. Allow enough space between each object.

## Duckiebot setup notes {#demo-projecttbd-duckiebot-setup}

No special setup needed for the duckiebot.

## Pre-flight checklist {#demo-projecttbd-pre-flight}

The pre-flight checklist for this demo are:
Check: Battery level is sufficient.
Check: Joystick container is running.
Check: Rospicam container is running.
Check: Put on your smartie pants.

## Demo instructions {#demo-projecttbd-run}

Here, give step by step instructions to reproduce the demo.

**Step 1**: When the duckiebot is powered on, make sure all the containers required are running. In your laptop, run

    laptop $ docker -H ![duckie_bot].local ps

to check whether the right containers are running or not. You can also check by going to the portainer webpage: http://![duckie_bot].local:9000/#/containers.

**Step 2**: Launch the rospicam container so that we can capture images from the camera of duckiebot.

    laptop $ docker -H ![duckie_bot].local run -it --name ros-picam --network=host  --device /dev/vchiq -v /data:/data
    duckietown/rpi-duckiebot-ros-picam:master18

This command will run a container and create a ros node inside automatically. This ros node will publish the images captured by the camera to a ros topic.

**Step 3**: Launch the object detector container.

Note: For this command you need the duckiebot's IP address. In order to obtain the duckiebot IP address, you should ping your duckiebot in another terminal and note down the IP address.

    laptop $ docker -H ![duckie_bot].local run -it --name object_detection --network=host -e ROS_MASTER_URI=http://duckie_bot_IP_Address:11311/  -e DUCKIEBOT_NAME=![duckie_bot] zgxsin/object_detection:1.6

Notice that we have to set up the ROS_MASTER_URI variable so that the ros nodes can communicate with each other. This command will create a object_detection ros node automatically. It will listen to the camera image topic in step 2 and predict images and send the predicted images to another topic for visualization.

**Step 4**: In another terminal type:

    laptop $ dts start_gui_tools ![duckie_bot]

We can check whether everything is working inside this container. (See Tip 1)
After that, run the following command in the container

    container $ rqt_image_view

This will pop up a new GUI window. Select the 'predicted images' topic from the drop down menu.

**Step 5**: Move the duckiebot using the joystick demo to different parts of duckietown and see the magic.  

## Troubleshooting {#demo-projecttbd-troubleshooting}

**Problem 1:** The duckiebot is not moving.
**Solution:** Make sure that the joystick container is running. Note that the command for launching the joystick was changed to
    laptop $ dts duckiebot keyboard_control ![duckie_bot]

**Problem 2:** No images recorded.
**Solution:** Make sure that the rospicam container is running.

**Problem 3:** The ros nodes cannot communicate with each other
**Solution:** If you are using docker on Mac OSX, there seems to be an issue with the network of docker containers. We recommend to use docker on Ubuntu 16.04. We have tested it and everything is fine.
(Insert Image)

**Problem 4:** The storage in the raspberry PI has reached its limit.
**Solution:** Run `docker -H ![duckie_bot].local images` to whether dangling images exist and run `docker -H ![duckie_bot].local  rmi --force image_ID` to remove them.

**Problem 5:** ERROR: unable to contact ROS master at [http://![duckiebot_name].local:11311/]
The traceback for the exception was written to the log file
**Solution:** make sure your laptop and duckiebot are on the same network

**Problem 6:** docker: Error response from daemon: Conflict. The container name "/object_detection" is already in use by container
**Solution:** Run the command in the laptop: docker container rm --force object_detection


**Tip 1**: We can check whether everything is working inside this container. Run `rosnode list` to check whether corresponding ros nodes are running. Run `rostopic list` to check whether the corresponding ros topics exist. You can run other ros-related command to do the check as well.


## Demo failure demonstration {#demo-projecttbd-failure}

Finally, put here a video of how the demo can fail, when the assumptions are not respected.
