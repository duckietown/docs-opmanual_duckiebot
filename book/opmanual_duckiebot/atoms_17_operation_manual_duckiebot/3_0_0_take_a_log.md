# Taking and verifying a log {#take-a-log status=outdated}

<div class='requirements' markdown='1'>

Requires: [](#read-camera-data)

Requires: [](#rc-control)

Result: A verified log.

</div>

## Preparation

Note: You may not have the folder. SSH into your robot and execute:

    duckiebot $ sudo mkdir /data/logs

Note: It is recommended but not required that you log to your USB and not to your SD card.

See: To mount your USB see [here](+software_reference#mounting-usb).


## Record the log {#record-log}

### Option: Minimal Logging on the Duckiebot

    laptop $ dts duckiebot demo --demo_name make_log_docker --duckiebot_name ![DUCKIEBOT_NAME] --package_name duckietown_demos --image duckietown/dt-core:daffy

This will only log the imagery, camera_info, the control commands and a few other essential things.


### Option: Full Logging on the Duckiebot

To log everything that is being published, run the base container on the Duckiebot:

    laptop $ dts duckiebot demo --demo_name make_log_full_docker --duckiebot_name ![DUCKIEBOT_NAME] --package_name duckietown_demos --image duckietown/dt-core:daffy


## Stop logging

You can stop the recording process by stopping the container:

    laptop $ docker -H ![DUCKIEBOT_NAME].local stop demo_make_log_docker
 
or `demo_make_log_full_docker` as the case may be. You can also do this through the portainer interface. 


## Getting the log

If you mounted a USB drive, you can unmount it and then remove the USB drive containing the logs (recommended).

See: For unmounting instructions see [here](+software_reference#mounting-usb)

Otherwise, if you are running the file server, you should see you logs at `http://![hostname].local:8082/logs/`


Otherwise you can copy the logs from your robot onto your laptop. Assuming they are on the same network execute:

    laptop $ scp ![linux_username]@![hostname].local:/data/logs/* ![path-to-local-folder]

You can also download a specific log instead of all by replacing `*` with the filename.

## Verify a log {#verify-a-log status=ready}

Either copy the log to your laptop or from within your container do

    $ rosbag info ![FULL_PATH_TO_BAG] --freq

Then:

- verify that the "duration" of the log seems "reasonable" - it's about as long as you ran the log command for

- verify that the "size" of the log seems "reasonable" - the log size should grow at about 220MB/min

- verify in the output that your camera was publishing very close to **30.0Hz** and verify that your virtual joysick was publishing at a rate of around **26Hz**.
