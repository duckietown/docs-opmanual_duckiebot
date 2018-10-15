# Camera troubleshooting {#setup-troubleshooting-camera status=draft}

Assigned: Russell

## Cannot see image.jpg in web browser

### Go to your Portainer.io and make sure the picam container is running
You can run with:

    laptop $ docker -H ![hostname].local run -d --name picam --device /dev/vchiq -v /data:/data duckietown/rpi-docker-python-picamera:master18

### Make sure dt18_01_health_stats_rpi-simple-server_1 container is running.
If it's not running something went wrong with your initialization. Try:

    laptop $ docker -H ![hostname].local run -dit --privileged --name dt18_01_health_stats_rpi-simple-server_1 --net host --restart unless-stopped duckietown/rpi-simple-server:master18

### Remove the battery pack and check the camera cable for damage. 
Some people bent the cable too much breaking it.

## You see a black image like this:

<figure id="Cap on photo">
    <figcaption>What you see if you leave the camera cap on.</figcaption>
     <img src="capon.png" style='width: 30em'/>
</figure>

### Resolution: Remove the cap.
