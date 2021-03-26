# Duckiebot V2 Indefinite Navigation {#dbv2-demos-in status=beta}

<div class='requirements' markdown='1'>

Requires: A Duckiebot fully setup and calibrated according to [](#dbv2)

Results: Duckiebot will perform indefinite navigation.

</div>

This demo is the same functionality as the indefinite navigation demo from DB18. It will do lane following,
intersection coordination and navigation, road sign april tag detection, etc. Similarly to the lane following
demo, this demo uses all of the same code as DB18, with the exception of the kinematics node, wheels driver node,
and lane controller node.

## Launching

Follow the procedure described in [](#dbv2-demos-launching), and then run this command:

    $ roslaunch duckietown_demos_dbv2 lane_following_dbv2.launch

## Issues

Some current prototypes of the DBV2 front bumper use a different LED from the usual DB18, and the LED
detector node does not properly detect it. The DBV2 will detect the DB18 and yield to it at intersections,
but the DB18 will _not_ detect the DBV2. The following figure shows that the DB18 could detect the DBV2 LEDs
if the blob size parameter was changed. However, this is currently hard-coded, and not a rosparam, so this would
require some refactoring and testing of the code.

<figure>
    <figcaption>View of a DBV2 from a DB18. If the LEDs were detected, this would show a red circle around them.</figcaption>
    <img src="dbv2_demos_images/front_on.png" style="width:100%"/>
</figure>

In addition to this issue, the DBV2 displays the same problems as DB18: It will sometimes over- or under-steer
when navigating through the intersection. This sometimes causes the Duckiebot to miss the next intersection. This
is show in the video below.

Another problem is that, when trying to drive straight, it will sometimes turn left instead.
This is because of the red-to-white process, which causes the duckiebot to interpret red stop lines as white lines.
This should therefore be disabled when driving straight.\

## Videos

TODO: Add the following videos:

 - `in_good.mp4`
 - `in-missedintersection.mp4`
 - `in-steeredwrong.mp4`
 - `in-understeer.mp4`