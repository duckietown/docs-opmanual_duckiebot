# Setting up the Docker workflow {#docker-setup status=ready}

Assigned: Andrea Censi

This section shows how to use the Docker functionality and introduces
some monitoring tools and workflow tips.


<div class='requirements' markdown="1">

Requires: You can ping and SSH into the robot, as explained in [](#setup-duckiebot).

Results: You have setup the Docker workflow.
 
</div>

## The Portainer interface {#docker-setup-portainer-interface}

Note: It makes sense to read this only once the network is established,
as explained in [](#setup-duckiebot). In particular, you need to be able to ping
and ssh to the robot.


Try to open the Portainer interface:

    http://![hostname].local:9000/#/containers
    
This will show the containers that are running.
 
### In case the above doesn't work

The first time that the Duckiebot starts it will download the containers to run.

Until everything is downloaded the Portainer interface will not start.

To debug this, login to the robot:

    laptop $ ssh ![Duckiebot name]

and then look at the logs:

    $ tail -f /var/log/syslog
    
You should see messages like this: 

    Sep 28 20:23:23 duckiebot cloud-init[695]: Loaded image: resin/raspberrypi3-alpine-python:slim
    Sep 28 20:23:25 duckiebot cloud-init[695]: Creating volume "local_data-volume" with local driver
    Sep 28 20:23:25 duckiebot cloud-init[695]: Pulling http-server (duckietown/rpi-simple-server:master18)...
    Sep 28 20:23:28 duckiebot cloud-init[695]: master18: Pulling from duckietown/rpi-simple-server
    Sep 28 20:24:17 duckiebot systemd[1]: Started Session c2 of user duckie.
    Sep 28 20:25:43 duckiebot cloud-init[695]: Digest: sha256:a0649d3e34176c84074d194905fd7e356d242f6100c47d3119d4202a1ea68aa3
    Sep 28 20:25:43 duckiebot cloud-init[695]: Status: Downloaded newer image for duckietown/rpi-simple-server:master18
    Sep 28 20:25:43 duckiebot cloud-init[695]: Pulling rpi-health (duckietown/rpi-health:master18)...
    Sep 28 20:25:45 duckiebot cloud-init[695]: master18: Pulling from duckietown/rpi-health
    Sep 28 20:26:46 duckiebot cloud-init[695]: Digest: sha256:f3bda06de3c0263113e4630618839a6af9613cbcc8f7b33facf06fab626b6c9a
    Sep 28 20:26:46 duckiebot cloud-init[695]: Status: Downloaded newer image for duckietown/rpi-health:master18
    Sep 28 20:26:46 duckiebot cloud-init[695]: Creating local_watchtower_1 ...
    Sep 28 20:26:46 duckiebot cloud-init[695]: Creating local_http-server_1 ...
    Sep 28 20:26:46 duckiebot cloud-init[695]: Creating local_rpi-health_1  ...
    Sep 28 20:26:46 duckiebot cloud-init[695]: Creating local_portainer_1   ...

Until you see `Creating ![container]` messages, the PI is still downloading the data.
 

## Communicating with Docker on the Duckiebot using the command line  {#docker-setup-communication}

The following commands can be run on your laptop but will affect the Duckiebot.

Note: It is never needed to log in to the Duckiebot via `ssh`, though that could be an alternative workflow.
 

You can set the variable `DOCKER_HOST` to point to the Duckiebot:

    laptop $ export DOCKER_HOST=![Duckiebot name].local
 
If you do, then you may omit every instance of the switch `-H ![Duckiebot name].local`.   

    
### Seeing which containers are running using `docker ps`

To test the connection, run `docker ps`:

    laptop $ docker -H ![Duckiebot name].local ps
    
    CONTAINER ID        IMAGE                                   ...
    84b6454111fd        resin/raspberrypi3-alpine-python:slim   ...
    c82d1487e2da        v2tec/watchtower:armhf-latest           ...
    dc34165b0e39        portainer/portainer:linux-arm           ...


This shows what containers are running on the Duckiebot. The information presented is
more limited than in Portainer.


<!--
 ### A third way: `ctop`

Another cool alternative is `ctop`, which you can install [from here][ctop-install].

[ctop-install]: https://github.com/bcicen/ctop
-->


## Health checks {#docker-setup-health-checks}

Warning: the container `duckietown/rpi-health` arrived only recently in the default config (Sep 27). If you have a previous SD card, you have to run it, using: 

    laptop $ docker -H ![Duckiebot name].local run --device /dev/vchiq -p 8085:8085 -d duckietown/rpi-health:master18

If some of the containers are marked as "unhealthy", fix the problem before continuing.

<figure id="portainer-unhealthy">
    <img src="portainer-unhealthy.png" style="width:100%"/>
    <figcaption>The portainer interface shows an "unhealthy" warning.</figcaption>
</figure>

In particular, the container `duckietown/rpi-health` checks some common hardware problems.

To access detailed information about the HW health, click the "logs" icon (second icon to the right of the orange "unhealthy" label). Alternatively, open the URL `http://![hostname].local:8085` in your browser.

Search for the `status` and `status_msgs` output:
    
    {
        "status": "error", 
        "status_msgs": [
            "Error: PI is throttled", 
            "Error: Under-voltage", 
            "Warning: PI throttling occurred in the past.", 
            "Warning: Under-voltage occurred in the past."
        ]
    ...
    }

The throttling and under-voltage warnings have to do with the power
supply. Note that the PI can be damaged by inadequate power supply,
so fix these as soon as possible.


## Seeing files on the Duckiebot {#docker-setup-simple-server}


On the Duckiebot there is a directory `/data` that will contain interesting files.

To access this content, you have two ways.

From another computer, you can see the contents of `/data` by visiting the URL:

    http://![hostname].local:8082
    
Otherwise, you can login via SSH and take a look at the contents of `/data`:

    laptop $ ssh ![hostname].local ls /data  
 


## Building workflow {#docker-setup-building-workflow}

Finally, we want to make sure that the Docker daemon on the robot
can build successfully.

To verify that, follow [the `rpi-duckiebot-simple-python` tutorial available here][here].


[here]: https://github.com/duckietown/rpi-duckiebot-simple-python
