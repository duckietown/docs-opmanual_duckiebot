# Camera troubleshooting {#setup-troubleshooting-camera status=draft}

Maintainer: Russell Buchanan

## Cannot see image.jpg in web browser

### Resolution: make sure the picam container is running
You can run with:

    laptop $ docker -H ![hostname].local run -d --name picam --device /dev/vchiq -v /data:/data duckietown/rpi-python-picamera:master18

### Resolution: make sure dt18_01_health_stats_rpi-simple-server_1 container is running.
If it's not running something went wrong with your initialization. Try:

    laptop $ docker -H ![hostname].local run -dit --privileged --name dt18_01_health_stats_rpi-simple-server_1 --net host --restart unless-stopped duckietown/rpi-simple-server:master18

However you likely either shutdown or disconnected it from the internet during initialization. The only fix is to re-flash the SD card.

### Resolution: remove the battery pack and check the camera cable for damage. 
Some people bent the cable too much breaking it.

## You see a black image like this:

<figure id="Cap on photo">
    <figcaption>What you see if you leave the camera cap on.</figcaption>
     <img src="capon.png" style='width: 30em'/>
</figure>

### Resolution: Remove the cap.


## Cannot change camera focus

### Need to break the glue
By default pi-cams come with the lens glued in place. Apply a bit more force the first time you adjust the lens.
## `libGL error` when running `dts start_gui_tools`

If you have an error like that when running `dts start_gui_tools` or another command with a GUI on the Duckiebot, then you are likely having issues with an NVIDIA graphics card:

`libGL error: No matching fbConfigs or visuals found libGL error: failed to load driver: swrast nvidia docker`

This could occur on a computer that has two grpahics cards: e.g. a discrete NVIDIA GPU, and an integrated Intel card. In order to run these commands you will have to switch to the Intel card. Please follow the official guidelines for your OS and grpahics card to find out how to do that.
