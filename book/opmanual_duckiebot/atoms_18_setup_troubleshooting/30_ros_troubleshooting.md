# ROS troubleshooting {#setup-troubleshooting-ros status=draft}

Assigned: Manish

## None of the ros command are recognised on duckiebot

Make sure that you are writing ros command, in root@hostname and not on the user@hostname

## Not able to connect to rosmaster

Load `hostname.local:9000` and check if roscore container is running.

## ROS pacakages of duckietown are not working on the laptop

Make sure that you source file named environment.sh before using ros commands. You can do that using `source environment.sh` on the command line ourside the catkin_ws folder but inside your duckietown repo.
`
