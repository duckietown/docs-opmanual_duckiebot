# ROS troubleshooting {#setup-troubleshooting-ros status=ready}

## None of the ROS command are recognized on Duckiebot

Make sure that you are writing ROS commands in `root@hostname` and not on the `user@hostname`.

## Not able to connect to `rosmaster`

Load `hostname.local:9000` and check if the `roscore` container is running.

## ROS pacakages of Duckietown are not working on the laptop

Make sure that you source file named `environment.sh` before using ROS commands. You can do that using `source environment.sh` on the command line outside the `catkin_ws` folder but inside your duckietown repository.
