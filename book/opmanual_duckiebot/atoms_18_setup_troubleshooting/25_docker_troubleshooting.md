# Docker troubleshooting {#setup-troubleshooting-docker status=ready}

## I stopped all the containers and now Portainer or other basic containers are not available

You need to `ssh` in your Duckiebot and start the containers manually.

Use `docker container list -a` to see its exact name and `docker start ![container_name]` to start it.

## I deleted all the containers

You need to `ssh` in your Duckiebot and re-create the containers.

Note that the containers have some special options to be given.

The configuration is described in the YAML files in `/var/local`, which currently are:

    /var/local/DT18_00_basic.yaml
    /var/local/DT18_01_health_stats.yaml
    /var/local/DT18_02_others.yaml
    /var/local/DT18_05_duckiebot_base.yaml

Each of this is in a [Docker compose][compose] format.

For example, `/var/local/DT18_00_basic.yaml` contains:

    version: '3'
    services:

      portainer:
        image: portainer/portainer:linux-arm
        command: ["--host=unix:///var/run/docker.sock", "--no-auth"]
        restart: always
        network_mode: "host"
        volumes:
          - /var/run/docker.sock:/var/run/docker.sock

      watchtower:
        image: v2tec/watchtower:armhf-latest
        command: ["--cleanup"]
        restart: always
        network_mode: "host"
        volumes:
          - /var/run/docker.sock:/var/run/docker.sock

You can now either run the container individually or use Docker compose.

Individually, you would copy the options:

    duckiebot $ docker run -d --restart always --network host -v /var/run/docker.sock:/var/run/docker.sock portainer/portainer:linux-arm --host=unix:///var/run/docker.sock --no-auth

With Docker compose you would use:

    duckiebot $ docker-compose -f /var/local/DT18_00_basic.yaml up


[compose]: https://docs.docker.com/compose/


## Container does not start  {#setup-troubleshooting-docker-starting status=ready}

Symptom: `docker: Error response from daemon: Conflict. The container name "/![container_name]" is already in use by container "![container_hash]". You have to remove (or rename) that container to be able to reuse that name.`

Resolution: Stop the container (`docker stop ![container_name]`) if running and then remove (`docker rm ![container_name]`) the container with the  

## Docker exits with `tls: oversized record received`

If Docker exits with the above error when running remote commands, the most likely reason is different versions of Docker on your computer and Duckiebot. You can check that by running `docker version` on both devices. If that is indeed the case, you need to upgrade the Docker binaries on your computer. To do that, follow the official instructions [here](https://docs.docker.com/install/linux/docker-ce/ubuntu/).

## I can't run a container because I get `exec user process caused "exec format error"`

An error like this:

`standard_init_linux.go:190: exec user process caused "exec format error"`

despite not being very descriptive typically means that there is a mismatch between the container's processor architecture and the one on your computer. Different processor architectures have different instruction sets and hence binaries compiled for one are generally not executable on another. Raspberry Pis use ARM processors, while most of the laptops use x86 architecture which makes them incompatible. Still, there's hope. Most of the Duckietown Raspberry Pi containers have a piece of magic inside called Qemu which allows emulation of an ARM processor on a x86 machine. You can activate this emulator if you change the default entrypoint of the container by adding `--entrypoint=qemu3-arm-static` to options when running it.
