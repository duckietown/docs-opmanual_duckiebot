# Taking and verifying a log {#take-a-log status=outdated}

<div class='requirements' markdown='1'>

Requires: [](#read-camera-data)

Requires: [](#rc-control))

Result: A verified log.

</div>

## Preparation

Note: You may not have the folder. SSH into your robot and execute:

    duckiebot $ sudo mkdir /data/logs

Note: It is recommended but not required that you log to your USB and not to your SD card.

See: To mount your USB see [here](+software_reference#mounting-usb).


## Record the log {#record-log}

    laptop $ dts duckiebot demo --demo_name make_log_docker --duckiebot_name ![DUCKIEBOT_NAME]

This will only log the imagery, camera_info, the control commands and a few other essential things.


### Option: Full Logging on the Duckiebot

To log everything that is being published, run the base container on the Duckiebot:

    laptop $ dts duckiebot demo --demo_name base --duckiebot_name ![DUCKIEBOT_NAME]

When inside run:

    duckiebot-container $ make log-full-docker

Note: You can always run the above command to get a terminal inside a container that has the base Duckietown software stack on the robot.


## Getting the log

If you mounted a USB drive, you can unmount it and then remove the USB drive containing the logs (recommended).

See: For unmounting instructions see [here](+software_reference#mounting-usb)

Otherwise, if you are running the file server, you should see you logs at `http://![hostname].local:8082/logs/`


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
