# See what your Duckiebot sees {#read-camera-data status=ready}

<div class='requirements' markdown='1'>

Requires: A Duckiebot in `DB18` or `DB-beta` configuration.

Requires: Laptop configured, according to [](#laptop-setup).

Requires: You have configured the Duckiebot as documented in [](#setup-duckiebot).

Result: You can see the output of the camera.

</div>

## Check the camera hardware

It might be useful to do a quick camera hardware check as documented in [](#howto-camera-db18).

## Viewing an Image Stream on Your Laptop {#view-image-using-rqt-image-view}

The imagery is streaming from your Duckiebot by default on startup.
To see it, run a base image on your laptop with:

    laptop $ dts start_gui_tools ![DUCKIEBOT_NAME]

Warning: Note that in here you input Duckiebot hostname, do not include `.local` part. 

The container will start. At the prompt, run:

    container $ rqt_image_view

The command should open a window where you can view the image.
You have to select the right topic from the dropdown menu:

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

Resolution: Sometimes window does not successfully spawn on the first try. You can Ctrl+c to terminate the prcess first, and try running that process again.

Symptom: `libGL error` when running `dts start_gui_tools`

Resolution: If you have an error like that when running `dts start_gui_tools` or another command with a GUI on the Duckiebot, then you are likely having issues with an NVIDIA graphics card:

`libGL error: No matching fbConfigs or visuals found libGL error: failed to load driver: swrast nvidia docker`

This could occur on a computer that has two graphics cards: e.g. a discrete NVIDIA GPU, and an integrated Intel card. In order to run these commands you will have to switch to the Intel card. Please follow the official guidelines for your OS and graphics card to find out how to do that.

Symptom: I don't see any image

Reolution: use `rostopic hz /![hostname]/camera_node/image/compressed` and see if the image is being published. Typically, the image publish rate should be near 30hz.

See also: For more information about `rostopic`, see [](#using-no-vnc).

Sumptom: My image topic is not being published.

Resolution: Check that the `duckiebot-interface` is running

Open [the Portainer interface](#docker-setup-portainer-interface) and check the running containers. You should see one called `duckiebot-interface`, using image `duckietown/dt-duckiebot-interface:daffy-arm32v7`.

You call also determine this by running:

    $ docker -H ![DUCKIEBOT_NAME].local ps

and look at the output to find the Duckiebot interface container and verify that it is running.

If that image is not running, you should manually start that or check to see if you `init_sd_card` procedure was correct.

To manually start the `duckiebot-interface`, do:

    laptop $ docker -H ![DUCKIEBOT_NAME].local run --name duckiebot-interface -v /data:/data --privileged --network=host -dit --restart unless-stopped duckietown/dt-duckiebot-interface:daffy-arm32v7

Symptom: The camera is not detected from duckiebot

Resolution: remove the battery pack and check the camera cable for damage. Some people bent the cable too much breaking it.

Symptom: Cannot change camera focus

Resolution: Need to break the glue
Some picams come with the lens glued in place. Apply a bit more force the first time you adjust the lens.


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


