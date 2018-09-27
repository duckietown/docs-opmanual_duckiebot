# See What Your Duckiebot Sees {#read-camera-data status=ready}


<div class='requirements' markdown='1'>



Requires: You have configured the Duckiebot. The procedure is documented in [](#setup-duckiebot).

</div>


## Check the camera hardware

It might be useful to do a quick camera hardware check.

See: The procedure is documented in [](#howto-mount-camera).



## Viewing a Single Image



### Start the HTTP File Server

on the duckiebot run:


    duckiebot $ docker run -d --name file-server -v /data:/data -p 8082:8082 duckietown/rpi-simple-server


Note: If you have the latest version of the SD card you don't need this because it's run on by default


### Create an image

Then write an image to file on the duckiebot:


    duckiebot $ docker run -d --name picam -v /data:/data -p 8081:8081 --privileged duckietown/rpi-docker-python-picamera


Then point your browser to `[!DUCKIEBOT_HOSTNAME].local:8081/image.jpg` and verify that it is the output from your camera. 

Now stop the `picam` container:


    duckiebot $ docker stop picam




## Viewing an Image Stream on Your Laptop

Start publishing images through ROS on the duckiebot:


    duckiebot $ docker run -it --name ros-picam --net host --privileged -v /data:/data duckietown/ros-picam 


 You should see a bunch of output that ends with:

```
WARN: [/duckiebot/cam_info_reader_node] ==============CompressedImage
INFO: [/duckiebot/camera_node] Initializing......
INFO: [/duckiebot/camera_node] ~framerate_high = 30 
INFO: [/duckiebot/camera_node] ~framerate_low = 15 
INFO: [/duckiebot/camera_node] ~res_w = 640 
INFO: [/duckiebot/camera_node] ~res_h = 480 
INFO: [/duckiebot/camera_node] Initialized.
INFO: [/duckiebot/camera_node] Start capturing.
INFO: [/duckiebot/camera_node] Published the first image.
```

Now on  your laptop run:


    laptop $ dts update
    laptop $ dts start_gui_tools ![DUCKIEBOT_NAME]


The container will start and then:


    laptop $ rqt_image_view


Should open a window where you can view the image (you have to select the right topic from the dropdown menu). 



## Verifying the Output

In a new terminal ssh into your robot and run the base image (if you don't have one running already):

#### Docker


    duckiebot $ docker run -it --net host --privileged base --name base duckiebot/rpi-duckiebot-base


#### ROS{status=deprecated}

simply ssh into your robot.

### List topics

You can see a list of published topics with the command:

    duckiebot $ rostopic list

See also: For more information about `rostopic`, see [](+software_reference#rostopic).

You should see the following topics:

    /![robot name]/camera_node/camera_info
    /![robot name]/camera_node/image/compressed
    /![robot name]/camera_node/image/raw
    /rosout
    /rosout_agg

### Show topics frequency

You can use `rostopic hz` to see the statistics about the publishing frequency:

    duckiebot $ rostopic hz /![robot name]/camera_node/image/compressed

On a Raspberry Pi 3, you should see a number close to 30 Hz:

    average rate: 30.016
        min: 0.026s max: 0.045s std dev: 0.00190s window: 841

### Show topics data

You can view the messages in real time with the command `rostopic echo`:

    duckiebot $ rostopic echo /![robot name]/camera_node/image/compressed

You should see a large sequence of numbers being printed to your terminal.

That's the "image" --- as seen by a machine.



