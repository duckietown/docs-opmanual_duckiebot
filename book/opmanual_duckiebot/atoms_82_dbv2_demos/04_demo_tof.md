# Duckiebot V2 Time of Flight {#dbv2-demos-tof status=beta}

<div class='requirements' markdown='1'>

Requires: A Duckiebot fully setup and calibrated according to [](#dbv2)

Results: Duckiebot will do lane following, and vehicle avoidance using the ToF sensors.

</div>

This demo replaces the vehicle avoidance control node. The original node uses the camera to detect the circle pattern
on the backs of each Duckiebot, and then uses this to control the speed of the Duckiebot to maintain a set distance.
This new node uses the Time of Flight sensors to do the same thing.

## Launching

Follow the procedure described in [](#dbv2-demos-launching), and then run this command:

    $ roslaunch duckietown_demos_dbv2 lane_following_dbv2.launch tof:=true

This can be combined with the ToF demo ([(#dbv2-demos-line-follower)]) by adding `line_follower:=true` to this command.

## Behavior

The DBV2 has 3 ToF sensors on the front, with the outer 2 angled outward to provide a wider total field of view. When
going around corners, the center sensor may lose sight of the front Duckiebot. To account for this, the current
version of this demo uses a weighted average of the measurements of the front 3 sensors: When turning left, more
weight is placed on the left sensor, and vice versa.

## Issues

When going around corners, the Duckiebot will still occasionally lose sight of the Duckiebot it is following, or
erroneously detect obstacles on the side of the road (for example, watchtowers).

## Videos

TODO: Add the ToF videos