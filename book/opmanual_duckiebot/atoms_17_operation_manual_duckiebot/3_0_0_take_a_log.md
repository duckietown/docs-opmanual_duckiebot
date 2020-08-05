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

    laptop $ dts duckiebot demo --demo_name make_log_docker --duckiebot_name ![DUCKIEBOT_NAME] --package_name duckietown_demos

This will only log the imagery, camera_info, the control commands and a few other essential things.

### Option: Full Logging on the Duckiebot

To log everything that is being published, run the base container on the Duckiebot:

    laptop $ dts duckiebot demo --demo_name make_log_full_docker --duckiebot_name ![DUCKIEBOT_NAME] --package_name duckietown_demos

## Stop logging

You can stop the recording process by stopping the container:

    laptop $ docker -H ![DUCKIEBOT_NAME].local stop demo_make_log_docker

or `demo_make_log_full_docker` as the case may be. You can also do this through the portainer interface.

## Getting the log

### Download through browser (Recommended)

Using the dashboard file tab, you can access files on your duckiebot. For more information about the files tab, see [here](#dashboard-tutorial-files). Go to `/logs` and you should see all your logs there. Simply click on the log you want to transfer to your computer and it will download through your browser.

If for some reason you are having trouble accesing dashboard, you can directly specify the webpage at `http://![hostname].local:8082/logs/`.

### Using a USB drive

If you mounted a USB drive, you can unmount it and then remove the USB drive containing the logs (recommended).

See: For unmounting instructions see [here](+software_reference#mounting-usb)

### Using SCP

Otherwise you can copy the logs from your robot onto your laptop. Assuming they are on the same network execute:

    laptop $ scp ![linux_username]@![hostname].local:/data/logs/* ![path-to-local-folder]

You can also download a specific log instead of all by replacing `*` with the filename.

## Verify a log {#verify-a-log status=ready}

Note: This procedure requires ROSBAG to be installed. If you have not installed that already, you can do so via:

    $ sudo apt-get install python-rosbag

Either copy the log to your laptop or from within your container do

    $ rosbag info ![FULL_PATH_TO_BAG] --freq

Then:

- verify that the "duration" of the log seems "reasonable" - it's about as long as you ran the log command for

- verify that the "size" of the log seems "reasonable" - the log size should grow at about 220MB/min

- verify in the output that your camera was publishing very close to **30.0Hz** and verify that your virtual joysick was publishing at a rate of around **26Hz**.

An example of the output looks like this:

    path:        avlduck2_2020-08-05-01-54-18.bag
    version:     2.0
    duration:    12.7s
    start:       Aug 04 2020 21:54:18.73 (1596592458.73)
    end:         Aug 04 2020 21:54:31.42 (1596592471.42)
    size:        69.9 MB
    messages:    756
    compression: none [77/77 chunks]
    types:       sensor_msgs/CameraInfo      [c9a58c1b0b154e0e6da7578cb991d214]
                sensor_msgs/CompressedImage [8f7a12909da2c9d3332d540a0977563f]
    topics:      /avlduck2/camera_node/camera_info        374 msgs @ 29.9 Hz : sensor_msgs/CameraInfo     
                /avlduck2/camera_node/image/compressed   382 msgs @ 29.8 Hz : sensor_msgs/CompressedImage
