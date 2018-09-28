# Setting up the Docker workflow {#docker-setup status=ready}

Assigned: Andrea Censi

This section shows how to use the Docker functionality and introduces
some monitoring tools and workflow tips.


## The Portainer interface {#docker-setup-portainer-interface}

First, open the Portainer interface:

    http://![hostname].local:9000/#/containers
    
This will show the containers that are running.
 
 

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

### A third way: `ctop`

Another cool alternative is `ctop`, which you can install [from here][ctop-install].

[ctop-install]: https://github.com/bcicen/ctop



## Health checks {#docker-setup-health-checks}

Warning: the container `duckietown/rpi-health` is not yet in the default config. Until it is you have to run it, using: 

    laptop $ docker -H ![Duckiebot name].local run --device /dev/vchiq -p 8085:8085 -d duckietown/rpi-health

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


## Building workflow {#docker-setup-building-workflow}

Finally, we want to make sure that the Docker daemon on the robot
can build successfully.

To verify that, follow [the `rpi-duckiebot-simple-python` tutorial available here][here].


[here]: https://github.com/duckietown/rpi-duckiebot-simple-python
