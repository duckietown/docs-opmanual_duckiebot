# Setting up the Docker workflow {#docker-setup status=ready}

This section shows how to use the Docker functionality and introduces
some monitoring tools and workflow tips. It is strongly recommended for you to take a read at [this](#preliminaries-docker-basics) section first.

<div class='requirements' markdown="1">

Requires: You have a basic understanding of docker as stated [here](#preliminaries-docker-basics)

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

    laptop $ ssh ![hostname]

and then run the following command:

    duckiebot $ docker ps

and look for the portainer container below. If you don't see that, that means your portainer container is not starting correctly and something is wrong.

    CONTAINER ID        IMAGE                             ...    Status             NAMES
    06d9d9529d0b        portainer/portainer:linux-arm     ...    Up 11 seconds      portainer
    ...                 ...                               ...    ...                ...

You should then refer

## Communicating with Docker on the Duckiebot using the command line {#docker-setup-communication}

The following commands can be run on your laptop but will affect the Duckiebot.

Note: It is never needed to log in to the Duckiebot via `ssh`, though that could be an alternative workflow.

### Seeing which containers are running using `docker ps`

To test the connection, run `docker ps`:

    laptop $ docker -H ![hostname].local ps

    CONTAINER ID        IMAGE                                   ...
    84b6454111fd        resin/raspberrypi3-alpine-python:slim   ...
    c82d1487e2da        v2tec/watchtower:armhf-latest           ...
    dc34165b0e39        portainer/portainer:linux-arm           ...

This shows what containers are running on the Duckiebot. The information presented is
more limited than in Portainer.

## Health checks {#docker-setup-health-checks}

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

## Docker common troubleshooting {#setup-troubleshooting-docker status=ready}

### docker: Got permission denied while trying to connect to the Docker daemon socket

If this is on your laptop, that means when you setup your enviornment you did not grant your user account right to do certain things. You can fix this by running:

    laptop $ sudo adduser `whoami` docker

Log out and in again and it should be fixed. 

### I stopped all the containers and now Portainer or other basic containers are not available

You need to `ssh` in your Duckiebot and start the containers manually.

Use `docker container list -a` to see its exact name and `docker start ![container_name]` to start it.

### I deleted all the containers

You need to `ssh` in your Duckiebot and re-create the containers.

Note that the containers have some special options to be given.

The configuration is described in the YAML files in `/data/config/autoboot`, which currently are:

    duckiebot.yaml
    traffic_light.yaml
    watchtower.yaml

Each of this is in a [Docker compose][compose] format.

You can now either run the container individually or use Docker compose.

Individually, you would copy the options:

    duckiebot $ docker run -d --restart always --network host -v /var/run/docker.sock:/var/run/docker.sock portainer/portainer:linux-arm --host=unix:///var/run/docker.sock --no-auth

With Docker compose you would use:

    duckiebot $ docker-compose -f /data/config/autoboot/duckiebot.yaml up

This way all the containers will be automatically recreated.

[compose]: https://docs.docker.com/compose/

### Container does not start {#setup-troubleshooting-docker-starting status=ready}

Symptom: `docker: Error response from daemon: Conflict. The container name "/![container_name]" is already in use by container "![container_hash]". You have to remove (or rename) that container to be able to reuse that name.`

Resolution: Stop the container (`docker stop ![container_name]`) if running and then remove (`docker rm ![container_name]`) the container with the

### Docker exits with `tls: oversized record received`

If Docker exits with the above error when running remote commands, the most likely reason is different versions of Docker on your computer and Duckiebot. You can check that by running `docker version` on both devices. If that is indeed the case, you need to upgrade the Docker binaries on your computer. To do that, follow the official instructions [here](https://docs.docker.com/install/linux/docker-ce/ubuntu/).

### I can't run a container because I get `exec user process caused "exec format error"`

An error like this:

`standard_init_linux.go:190: exec user process caused "exec format error"`

despite not being very descriptive typically means that there is a mismatch between the container's processor architecture and the one on your computer. Different processor architectures have different instruction sets and hence binaries compiled for one are generally not executable on another. Raspberry Pis use ARM processors, while most of the laptops use x86 architecture which makes them incompatible. Still, there's hope. Most of the Duckietown Raspberry Pi containers have a piece of magic inside called Qemu which allows emulation of an ARM processor on a x86 machine. You can activate this emulator if you change the default entrypoint of the container by adding `--entrypoint=qemu3-arm-static` to options when running it.

## Recreate all container at once {#dt-manualupdate status=beta}

Instead of automagically updating the container using `dts`, you can also update the container manually using docker commands. SSH into your duckiebot, and then run:

    duckiebot $ dt-autoboot

This will recreate the all the container image.

## Run a specific image from boot {#dt-update-image status=beta}

To run a specific container, you can run:

    laptop $ docker run -H ![DUCKIEBOT_NAME].local -restart=always --net=host -v /data:/data -v /var/run/avahi-daemon/socket:/var/run/avahi-daemon/socket ![ORG/NAME:TAG]
