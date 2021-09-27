# Duckiebot V2 Line Follower {#dbv2-demos-line-follower status=beta}

<div class='requirements' markdown='1'>

Requires: A Duckiebot fully setup and calibrated according to [](#dbv2)

Results: Duckiebot will follow the lane using the line follower sensors.

</div>

This demo replaces the lane controller node with one that uses only the line following sensors on the front
bumper of the DBV2. It finds the white line on the right side of the road, and follows the border between the
white line and the black road surface.

It uses a PI controller for steering, and maintains a constant forward speed. It is generally required to move more
slowly than normal camera-based lane following, which is the case with the default parameters.

## Launching

Follow the procedure described in [](#dbv2-demos-launching), and then run this command:

    $ roslaunch duckietown_demos_dbv2 lane_following_dbv2.launch line_follower:=true

This can be combined with the ToF demo ([(#dbv2-demos-tof)]) by adding `tof:=true` to this command.

## Videos

TODO: Add the two line follower videos