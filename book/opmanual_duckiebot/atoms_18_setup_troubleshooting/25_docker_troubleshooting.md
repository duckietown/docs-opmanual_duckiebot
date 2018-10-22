# Docker troubleshooting {#setup-troubleshooting-docker status=ready}

Assigned: OPEN



## I stopped all the containers and now Portainer is not available
You need to `ssh` in your Duckiebot and start the Portainer container manually. Use `docker container list -a` to see its exact name and `docker start ![container_name]` to start it.

## Container does not start  {#setup-troubleshooting-docker-starting}

Symptom: `docker: Error response from daemon: Conflict. The container name "/![container_name]" is already in use by container "![container_hash]". You have to remove (or rename) that container to be able to reuse that name.`

Resolution: Stop (`docker stop ![container_name]`) if running and then remove (`docker rm ![container_name]`) the container with the  

## Docker exits with tls: oversized record received

If Docker exits with the above error when running remote comamnds, the most likely reason is different versions of Docker on your computer and Duckiebot. You can check that by running `docker version` on both devices. If that is indeed the case, you need to upgrade the Docker binaries on your computer. To do that, follow the official instructions [here](https://docs.docker.com/install/linux/docker-ce/ubuntu/).
