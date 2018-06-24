# Taking and verifying a log {#take-a-log status=outdated}

<div class='requirements' markdown='1'>

Requires: [](#read-camera-data)

Requires: [](#rc-control))

Result: A verified log.

</div>

## Preparation

Note: it is recommended that you log to your USB and not to your SD card.

See: To mount your USB see [](+software_reference#mounting-usb).

## Run something on the Duckiebot

For example, if you want to drive the robot around and collect image data you could run:

    duckiebot $ make demo-joystick-camera

But anything could do.

## View images on the laptop

Run on the laptop:

    laptop $ cd ![Duckietown root]
    laptop $ source environment.sh
    laptop $ source set_ros_master.sh ![robot name]
    laptop $ rqt_image_view

and verify that indeed your camera is streaming imagery.

## Record the log {#record-log}

### Option: Full Logging

To log everything that is being published, on the Duckiebot in a new terminal (See [](+software_reference#byobu)):

    duckiebot $ make log-full

where here we are assuming that you are logging to the USB and have followed [](+software_reference#mounting-usb).

### Option: Log Minimal

To log only the imagery, camera_info, the control commands and a few other essential things, on the Duckiebot in a new terminal (See [](+software_reference#byobu)):

    duckiebot $ make log-minimal

where here we are assuming that you are logging to the USB and have followed [](+software_reference#mounting-usb).


## Verify a log {#verify-a-log status=beta}


On the Duckiebot run:

    duckiebot $ rosbag info ![FULL_PATH_TO_BAG] --freq

Then:

- verify that the "duration" of the log seems "reasonable" - it's about as long as you ran the log command for

- verify that the "size" of the log seems "reasonable" - the log size should grow at about 220MB/min

- verify in the output that your camera was publishing very close to **30.0Hz** and verify that you joysick was publishing at a rate between **3Hz** and **6Hz**.

TODO for Andrea Censi: More complex log verification methods.
