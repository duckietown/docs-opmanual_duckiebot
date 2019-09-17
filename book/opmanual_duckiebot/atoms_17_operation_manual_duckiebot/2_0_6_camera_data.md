# See what your Duckiebot sees {#read-camera-data status=ready}


<div class='requirements' markdown='1'>

Requires: A Duckiebot in `DB18` configuration.

Requires: Laptop configured, according to [](#laptop-setup).

Requires: You have configured the Duckiebot as documented in [](#setup-duckiebot).

Result: You can see the output of the camera.

</div>


## Check the camera hardware

It might be useful to do a quick camera hardware check as documented in [](#howto-mount-camera).


## Viewing an Image Stream on Your Laptop {#view-image-using-rqt-image-view}


The imagery is streaming from your Duckiebot by default on startup.
To see it, run a base image on your laptop with:

    laptop $ dts start_gui_tools ![DUCKIEBOT_NAME] --base_image duckietown/dt-core:daffy-amd64


The container will start. At the prompt, run:


    container $ rqt_image_view


The command should open a window where you can view the image.
You have to select the right topic from the dropdown menu:

<figure>
    <figcaption>The rqt image view window with dropdown menu</figcaption>
    <img style='width:12em' src="rqt_image_view.png"/>
</figure>


### Troubleshooting

Symptom: I don't see any image

Reolution: Check that the `duckiebot-interface` is running

Open [the Portainer interface](#docker-setup-portainer-interface) and check the running containers. You should see one called `dt18_03_roscore_duckiebot-interface_1`.

You call also determine this by running:

    $ docker -H ![DUCKIEBOT_NAME].local ps

and look at the output to find the Duckiebot interface container and verify that it is running.


## Viewing the image stream on the Dashboard {#image-dashboard status=beta}

If you followed the instructions in [](#duckiebot-dashboard-setup), you
should have access to the Duckiebot dashboard.

Open the browser and visit the page `http://![hostname].local/mission-control`.

The bottom of the page shows the camera block.
You should be able to see the camera feed in the camera block,
as shown in the image below.

<div figure-id="fig:dashboard_mission_control_camera_feed" figure-caption="">
  <img src="dashboard_mission_control_camera_feed.png" style='width: 35em'/>
</div>

By default, the camera stream is throttled down to 8 frames per second.
This is to minimize the resources used by your browser while streaming
images from the robot.
Feel free to increase the data stream frequency in the **Properties** tab
of the camera block.

Note: If you see a black image in the camera block, make sure that you
removed the protective cap that covers the camera lens of your Duckiebot.



## Verifying the output by using the ROS utilities

Use the commands below to check the data streams in ROS.


### List topics

You can see a list of published topics with the command:

    container $ rostopic list

See also: For more information about `rostopic`, see [](+software_reference#rostopic).

You should see at least the following topics:

    /![hostname]/camera_node/camera_info
    /![hostname]/camera_node/image/compressed
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
