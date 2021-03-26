# Duckiebot V2 Lane Following {#dbv2-demos-lf status=beta}

<div class='requirements' markdown='1'>

Requires: A Duckiebot fully setup and calibrated according to [](#dbv2)

Results: Duckiebot will perform lane following using the camera.

</div>

DBV2 can do the same lane following demo as DB18: Using only the camera and no additional sensors,
it can follow a lane. It also has vehicle detection/avoidance using the circle pattern on the backs
of the Duckiebots.

This demo uses almost entirely the same code as DB18, with the exception of the kinematics node, wheels driver,
and lane controller node. If the wheel encoder is present, the kinematics node will use it for closed-loop speed
control, as described in [](#dbv2-calib-kinematics). To disable this, you can unplug the wheel encoder, or
stop the encoder node.

## Launching

Follow the procedure described in [](#dbv2-demos-launching), and then run this command:

    $ roslaunch duckietown_demos_dbv2 lane_following_dbv2.launch

## Videos and Images

TODO: Add video `lane_following.mp4`

<figure>
    <figcaption>Graph of demanded speed vs. speed measured by the wheel encoder</figcaption>
    <img style="width:100%" src="dbv2_demos_images/velocity_graph.png" />
</figure>