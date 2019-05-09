# Containers troubleshooting {#trouble-demo-containers status=draft}

Sometimes you need to access a container (i.e. a shell inside of it).

## Access trough Portainer
To do so you can either use Portainer, and from there select the container you are interested in and click on `>_ Console` and then `Connect`.

## Access trough terminal
A simple alternative is to Portainer is to directly use the terminal. Once your demo container is running, you can execute:

    laptop $ docker -H duckiebot.local exec -it demo_![demo_name]

To open a new shell in the container that is running your demo. From there you can for example use all the normal `ROS` commands, such as `rosnode list`.
If instead you are interested on the output of the `ROS` launch procedure, you can access the shell in which the launch file is executed by running:

    laptop $ docker -H duckiebot.local attach demo_![demo_name]

Furthermore, if your container crashes and you want to discover why, it is useful to access the logs, this can be done trough:

    laptop $ docker -H duckiebot.local logs ![container_name]
