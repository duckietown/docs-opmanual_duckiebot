# See what your Duckiebot sees {#read-camera-data status=ready}


<div class='requirements' markdown='1'>

Requires: You have configured the Duckiebot as documented in [](#setup-duckiebot).

Requires: You have configured Docker communication as documented in [](#docker-setup).

</div>


## Check the camera hardware

It might be useful to do a quick camera hardware check.

See: The procedure is documented in [](#howto-mount-camera).


## Viewing a Single Image


These commands assume that you have completed the steps in [](#docker-setup),
and in particular that you set `DOCKER_HOST` correctly and can use `docker ps` successfully.

Start the container `rpi-docker-python-picamera`. It reads the camera
image and writes it to `/data`.

    laptop $ docker -H ![hostname].local run -d --name picam --device /dev/vchiq -v /data:/data duckietown/rpi-docker-python-picamera:master18

Then point your browser to the address

    http://![hostname].local:8082/image.jpg

and verify that it is the output from your camera.

If the image is all black, check that you have removed the lens cap.


Now stop the `picam` container:

    laptop $ docker -H ![hostname].local stop picam



## Viewing an Image Stream on Your Laptop

Start publishing images through ROS on the Duckiebot using
the container `rpi-duckiebot-ros-picam`:


    laptop $ docker -H ![hostname].local run -it --name ros-picam --network=host  --device /dev/vchiq -v /data:/data  duckietown/rpi-duckiebot-ros-picam:master18 

Note: you need `-v /data:/data` because of the calibration procedure later.

You should see of output that ends with:

    WARN: [/duckiebot/cam_info_reader_node] ==============CompressedImage
    INFO: [/duckiebot/camera_node] Initializing......
    INFO: [/duckiebot/camera_node] ~framerate_high = 30
    INFO: [/duckiebot/camera_node] ~framerate_low = 15
    INFO: [/duckiebot/camera_node] ~res_w = 640
    INFO: [/duckiebot/camera_node] ~res_h = 480
    INFO: [/duckiebot/camera_node] Initialized.
    INFO: [/duckiebot/camera_node] Start capturing.
    INFO: [/duckiebot/camera_node] Published the first image.


Now start the GUI tools container in a different terminal:


    laptop $ dts start_gui_tools ![hostname]


The container will start. At the prompt, run:


    container $ rqt_image_view


The command should open a window where you can view the image.
You have to select the right topic from the dropdown menu.

## Verifying the output by using the ROS utilities

Close the `rqt_image_view` window and type the next commands in the same
window.


### List topics

You can see a list of published topics with the command:

    container $ rostopic list

See also: For more information about `rostopic`, see [](+software_reference#rostopic).

You should see at least the following topics:

    /![hostname]/camera_node/camera_info
    /![hostname]/camera_node/image/compressed
    /![hostname]/camera_node/image/raw
    /rosout
    /rosout_agg

There might be other topics if you started other demos.

### Show topics frequency

You can use `rostopic hz` to see the statistics about the publishing frequency:

    container $ rostopic hz /![hostname]/camera_node/image/compressed

On a Raspberry Pi 3, you should see a number close to 30 Hz:

    average rate: 30.016
        min: 0.026s max: 0.045s std dev: 0.00190s window: 841

Use CTRL-C to stop `rostopic`.

### Show topics data

You can view the messages in real time with the command `rostopic echo`:

    container $ rostopic echo /![hostname]/camera_node/image/compressed

You should see a large sequence of numbers being printed to your terminal.

That's the "image" --- as seen by a machine.
