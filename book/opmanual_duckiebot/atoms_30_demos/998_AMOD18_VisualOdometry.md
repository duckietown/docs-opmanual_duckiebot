# AMOD18 Visual Odometry {#demo-visualodometry status=draft}

This is the report of the AMOD18 Visual Odometry group. In the first section we present how to run the demo, while the second section is dedicated on a more complete report of our work.

## The visual odometry demo

<div class='requirements' markdown="1">

Requires: 1 Duckiebot in configuration DB18

Requires: 1 External computer to operate the demo.

Requires: Camera calibration completed.

</div>

### Video of expected results {#demo-visualodometry-expected}

First, we show a video of the expected behavior (if the demo is succesful).

### Duckietown setup notes {#demo-visualodometry-duckietown-setup}

To run this demo, you can setup any generic Duckietown which complies to the appereance specifications presented in [](+opmanual_duckietown#duckietown-specs).  

The only two constraints are that you have good lighting conditions and enough possible features in the field of view of the Duckiebot (they can be anything, really, from duckies to street signs to a replica of the Saturn V).

Note: Most of your environment should be static. Otherwise you might get bad results.

### Pre-flight checklist {#demo-visualodometry-pre-flight}

The pre-flight checklist describes the steps that are sufficient to
ensure that the demo will be correct:

Check: Your Duckiebot has enough battery to run a demo.  

Check: Your performed recently the camera calibration on your Duckiebot.  



### Demo instructions {#demo-visualodometry-run}

Step 1: From your computer run the demo container on your duckiebot typing the command:

    laptop $ do something here

Step 2: On your computer open a virtual joystick to steer the bot

Step 3: Open `rviz` by running:

    laptop $ some meaningful command

Step 4: Start your Duckiebot by pressing -<kbd>a</kbd> in the virtual joystick


### Troubleshooting {#demo-visualodometry-troubleshooting}

Symptom: The duckiebot does not move.

Resolution: Make sure you tried the virtual joystick demo

Symptom: The estimated pose is really bad.

Resolution: You might have a too dynamic scene, for the visual odometry to run correctly.

### Demo failure demonstration {#demo-visualodometry-failure}

Finally, put here a video of how the demo can fail, when the assumptions are not respected.


## The AMOD18 Visual Odometry final report

### Mission and Scope

Our goal is to provide an estimate of the pose, using a monocular visual odometry approach.

#### Motivation

Providing a pose estimate is essential in the operation of a Duckiebot (how can you go where you want if you don't know where you are), and yet having only one camera it is a non trivial task and could yield catastrophic results if fails.

#### Existing solution

The current solution for the pose estimate is the so called `Lane Filter`, which estimates the pose of the Duckiebot with respect of a template straight tile.

The pose is estimate by a distance $d[m]$ from the center of the lane, and an angle $\phi[rad]$ which is measured with respect to the direction of the lane.

#### Opportunity

The `Lane Filter` assumes that the Duckiebot always travel along a straight road, which is clearly not possible, because Duckietown is full of exciting curves and interesting intersections.

Therefore, by applying a working visual odometry pipeline, it would be possible to get an estimate about the pose even independantly on where the Duckiebot is travelling.

### Definition of the problem

#### Final objective

Given a Duckiebot travelling in Duckietown, an estimate of the pose with respect to a relevant frame should be obtained:

* At a frequency of ~12 Hz on a Raspberry Pi
* With an accuracy similar to the the one provided by the `Lane Filter`, with possibility of drift and divergence in long-term executions

#### Assumptions

The following assumptions are made:

* Duckiebot in configuration DB18
* The camera is calibrated and works properly (30Hz and 640x480 resolution)
* Most of the scene is static

### Added functionality

#### Visual odometry

The algorithm performes a visual odometry pipeline as follows.  
First, the image get downsampled (by half), to decrease the computational time. Then feature detection is performed using XXX. On the next image frame, the feature detection is again performed, then the features are matched using XXX. To remove outliers that could generate significant errors XXX is used. After a reliable set of pairs is obtained, the motion between the frames is extracted by XXX. By multiplying the current pose estimate with the obtained matrix, a new estimate of the pose is received.

#### Software architecture

Nodes:  

visual_odometry_node:  

  * Input: The compressed camera image

  * Output: A pose estimate with respect to a world frame

    Subsribed topics:

      * camera_node/image_compressed

      * fsm_node/switch

    Published topics:

      * path

### Future development

The pipeline can be improved by performing bundle adjustment, however the computational burdain in that case is unclear. Other functionalities which could be interesting are a controller based on the visual odometry to travel intersections, and the placement of the path inside a given map.
