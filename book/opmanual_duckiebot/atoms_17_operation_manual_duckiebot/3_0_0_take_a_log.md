# Taking and verifying a log {#take-a-log status=outdated}

<div class='requirements' markdown='1'>

Requires: [](#read-camera-data)

Requires: [](#rc-control))

Result: A verified log.

</div>

## Preparation

Note: If you didn't recently burn your SD card, you may not have the folder. SSH into your robot and execute:

    duckiebot $ sudo mkdir /data/logs

Note: It is recommended but not required that you log to your USB and not to your SD card.

See: To mount your USB see [here](http://docs.duckietown.org/DT18/software_reference/out/mounting_usb.html).


## Run something on the Duckiebot

For example, if you want to drive the robot around and collect image data. For example run [Lane Following Demo](#demo-lane-following) or the camera and the joystick.

View images on the laptop and verify that indeed your camera is streaming imagery See [](#read-camera-data).

## Record the log {#record-log}

    laptop $ docker -H ![hostname].local run -it --net host --name logger -v /data/logs:/logs duckietown/rpi-duckiebot-logger:master18

This will only log the imagery, camera_info, the control commands and a few other essential things.

### Option: Full Logging

To log everything that is being published, run the above docker container but put `/bin/bash` at the end. When inside run

    duckiebot-container $ make log-full-docker


## Getting the log

If you mounted a USB drive, you can unmount it and then remove the USB drive containing the logs (recommended).

See: For unmounting instructions see [here](http://docs.duckietown.org/DT18/software_reference/out/mounting_usb.html).

Otherwise, if you are running the file server, you should see you logs at `http://![hostname].local:8082/logs/`

Doubt: I see the folder but not the logs why ?

Otherwise you can copy the logs from your robot onto your laptop. Assuming they are on the same network execute:

    laptop $ scp ![hostname]/data/logs/* ![path-to-local-folder]

You can also download a specific log instead of all by replacing `*` with the filename.

## Verify a log {#verify-a-log status=ready}

Either copy the log to your laptop or from within your container do

    $ rosbag info ![FULL_PATH_TO_BAG] --freq

Then:

- verify that the "duration" of the log seems "reasonable" - it's about as long as you ran the log command for

- verify that the "size" of the log seems "reasonable" - the log size should grow at about 220MB/min

- verify in the output that your camera was publishing very close to **30.0Hz** and verify that you joysick was publishing at a rate between **3Hz** and **6Hz**.

TODO for Andrea Censi: More complex log verification methods.
