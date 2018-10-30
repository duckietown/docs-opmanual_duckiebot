# Docker troubleshooting {#setup-troubleshooting-docker status=ready}

Assigned: OPEN



## I stopped all the containers and now Portainer is not available


## Container does not start  {#setup-troubleshooting-docker-starting}

Symptom: `docker: Error response from daemon: Conflict. The container name "/![container_name]" is already in use by container "![container_hash]". You have to remove (or rename) that container to be able to reuse that name.`

Resolution: Stop (`docker stop ![container_name]`) if running and then remove (`docker rm ![container_name]`) the container with the  
