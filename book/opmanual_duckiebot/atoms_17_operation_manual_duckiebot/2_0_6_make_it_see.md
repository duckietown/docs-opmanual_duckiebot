# Operation - Make it see {#read-camera-data status=ready}

<div class='requirements' markdown='1'>

Requires: A Duckiebot in `DB18` or above configuration.

Requires: Laptop configured, according to [](#laptop-setup).

Requires: You have configured the Duckiebot as documented in [](#setup-duckiebot).

Result: You can see the output of the camera.

</div>

This section describes how to see what your Duckiebot sees.

## Check the camera hardware

If you have a `DB18` or `DB19`, it might be useful to do a quick camera hardware check as documented in [](#howto-camera-db18).

## Viewing an Image Stream on Your Laptop {#view-image-using-rqt-image-view}

The imagery is streaming from your Duckiebot by default on startup.
To see it, open a terminal on your laptop and run:

    laptop $ dts start_gui_tools ![DUCKIEBOT_NAME]

Warning: Note that in here you input Duckiebot ![hostname], do not include `.local` part.

This will start a container with access to the ROS messages of the Duckiebot. At the prompt, run:

    container $ rqt_image_view

The command will open a window where you can view the image.

You have to select the right topic from the drop-down menu:

<figure>
    <figcaption>The rqt image view window with dropdown menu</figcaption>
    <img style='width:12em' src="rqt_image_view.png"/>
</figure>

### Troubleshooting

Symptom: I see a black image like this:

<figure id="Cap on photo">
    <figcaption>What you see if you leave the camera cap on.</figcaption>
     <img src="capon.png" style='width: 30em'/>
</figure>

Resolution: Remove the cap of the camera.

Symptom: When I try to do `rqt_image_view`, I don't see the window on my machine.

Resolution: Sometimes the window does not successfully spawn on the first try. You can <kbd>Ctrl</kbd>+<kbd>c</kbd> to terminate the process before trying again.

Symptom: `libGL error` when running `dts start_gui_tools`

Resolution: If you have an error like that when running `dts start_gui_tools` or another command with a GUI on the Duckiebot, then you are likely having issues with an NVIDIA graphics card:

`libGL error: No matching fbConfigs or visuals found libGL error: failed to load driver: swrast nvidia docker`

This could occur on a computer that has two graphics cards, e.g., a NVIDIA GPU and an integrated Intel card. Switch to the Intel card by following the official guidelines for your OS and graphics card.

Symptom: I don't see any image.

Resolution: use `rostopic hz /![hostname]/camera_node/image/compressed` and see if the image is being published. Images should be published at roughly 30 Hz.

See also: For more information about `rostopic`, see [](#using-no-vnc).

See also: you can see the images as your robot sees them with `rostopic echo /![hostname]/camera_node/image/compressed`. <kbd>Ctrl</kbd>+<kbd>c</kbd> on the terminal once you've seen enough.

Symptom: My image topic is not being published.

Resolution: Check that the `duckiebot-interface` is running

Open [the Portainer interface](#sub:dashboard-portainer) and check the running containers. You should see one called `duckiebot-interface`, using image `duckietown/dt-duckiebot-interface:daffy-arm32v7`.

You call also determine this by running:

    $ docker -H ![DUCKIEBOT_NAME].local ps

and look at the output to find the Duckiebot interface container and verify that it is running.

If that image is not running, you should manually start that or check to see if you `init_sd_card` procedure was correct.

To manually start the `duckiebot-interface`, do:

    laptop $ docker -H ![DUCKIEBOT_NAME].local run --name duckiebot-interface -v /data:/data --privileged --network=host -dit --restart unless-stopped duckietown/dt-duckiebot-interface:daffy-arm32v7

Symptom: The camera is not detected from Duckiebot

Resolution: (`DB18`, `DB19` only) remove the battery pack and check the camera cable for damage.

Symptom: The images are out of focus.

The camera focus can be _manually_ adjusted by rotating the lens of the image sensor. As always with dealing with hardware, exercise care and do not use force.  

Symptom: I cannot rotate the lens and change the camera focus.

Resolution: You need to break the glue. Very occasionally cameras come with the lens glued in place. Apply a bit more force the first time you adjust the lens to break the glue's adhesion.


## Viewing the image in no-vnc {#image-novnc status=ready}

For instructions using the no-vnc tool, see [here](#using-no-vnc)

## Viewing the image stream on the Dashboard {#image-dashboard status=ready}

If you followed the instructions in [](#duckiebot-dashboard-setup), you
should have access to the Duckiebot dashboard.

Open the browser and visit the page `http://![hostname].local/`. Login using your duckietown token, and select robot panel on the left hand side navigation bar. Once selected you should see mission control page there. If you are unfamiliar with the dasboard, you can find more information here: [](#dashboard-overview)

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
