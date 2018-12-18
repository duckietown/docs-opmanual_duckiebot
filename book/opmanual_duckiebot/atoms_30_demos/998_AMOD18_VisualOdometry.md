# AMOD18 Visual Odometry {#demo-visualodometry status=draft}

This is the report of the AMOD18 Visual Odometry group. In the first section we present how to run the demo, while the second section is dedicated on a more complete report of our work.

## The visual odometry demo

<div class='requirements' markdown="1">

Requires: 1 Duckiebot in configuration [DB18](#duckiebot-configurations) (make sure there is a Duckie on it)

Requires: 1 External computer to operate the demo, with a [Docker installation](#laptop-setup)

Requires: [Camera calibration](#camera-calib) completed.

Requires: [Joystick demo](#rc-control) succesful.

</div>

### Video of expected results {#demo-visualodometry-expected}

First, we show a video of the expected behavior (if the demo is succesful).

### Duckietown setup notes {#demo-visualodometry-duckietown-setup}

To run this demo, you can setup any generic Duckietown which complies to the appereance specifications presented in [the duckietown specs](+opmanual_duckietown#duckietown-specs).  

The only two constraints are that you have good lighting conditions and enough texture in the field of view of the Duckiebot (it can be anything, really, from duckies to street signs to a replica of the Saturn V).

Note: Most of your environment should be static. Otherwise you might get bad results.

### Pre-flight checklist {#demo-visualodometry-pre-flight}

The pre-flight checklist describes the steps that are sufficient to
ensure that the demo will be correct:

Check: Your Duckiebot has enough battery to run a demo.  

Check: Your performed recently the camera calibration on your Duckiebot.  

Check: Both yout duckiebot and your laptop are connected to the same, stable network

### Demo instructions {#demo-visualodometry-run}

Step 1: From your computer load the demo container on your duckiebot by typing the command:

    laptop $ docker -H ![hostname].local run -it --net host --memory="800m" ---memory-swap="1.8g" --privileged -v /data:/data --name visual_odometry_demo  ![unclear]/visualodo-demo:master18


Step 2: Start the graphical user interface container:

    laptop $ dts start_gui_tools ![hostname]

Step 3: Download the rviz configuration file `odometry_rviz_conf` from our repo to your laptop-container by running:

    laptop-container $ wget https://raw.githubusercontent.com/duckietown/duckietown-visualodo/master/odometry_rviz_conf.rviz

Step 4: Check that you can visualize the list of topics in the duckiebot from the laptop:

    laptop-container $ rostopic list

Step 5: On the same terminal, run `rviz` with the downloaded configuration file

    laptop-container $ rosrun rviz rviz -d ![path_to_file]/odometry_rviz_conf.rviz

Step 6: On a new terminal in the computer, open a virtual joystick to steer the bot

    laptop $ dts duckiebot keyboard_control ![hostname]

Step 7: Start your Duckiebot by pressing <kbd>a</kbd> in the virtual joystick.

Step 8: Be amazed!


### Troubleshooting {#demo-visualodometry-troubleshooting}

Symptom: The duckiebot does not move.

Resolution: Make sure you tried the virtual joystick demo

Symptom: The estimated pose is really bad.

Resolution: You might have a too dynamic scene, for the visual odometry to run correctly.

Symptom: The estimated pose is really bad and the scene is not dynamic

Resolution: Debug the pipeline by turning on the plotting parameters

*Add videos here*

### Demo failure demonstration {#demo-visualodometry-failure}

Finally, put here a video of how the demo can fail, when the assumptions are not respected.


## The AMOD18 Visual Odometry final report {#final-report-visualodo}

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

When the `visual_odometry_node` receives an image, the relative callback function gets activated. If the node should be active according to the `FSM`, the pipeline starts.

The image gets converted to an `OpenCV` object. Then the actual algorithm present in `lib-visualodo` is triggered with this image object.  

The image gets then downsampled, as this reduces significantly the computational time. Then relevant features are extracted from the image, using either `SURF`, `SIFT` or `ORB` (by default `ORB` is chosen).  

At each frame, we gather one image and we discard the oldest one, so to keep always the two most recent to perform the pipeline.  

Then we need to match the features, with either `KNN` or using the Hamming distance (default). If KNN is chosen, its matches can be filtered using a first-neighbor-to-second-neighbor ratio threshold. Empirically Bruteforce Hamming distance has proven to outperform KNN in the duckietown environment.

Matches may be further filtered using histogram fitting (activated by default, can be turned off). This means that we fit a gaussian distribution to the lenght and angle of the matches, and we remove the ones further than `x` standard deviations from the average value. These `x` values can be set in the parameters yaml file.

Then we divide the feature pairs between far and close regions, to decouple the estimate of the translation vector to the estimate of the rotation matrix (BangleiGuan et.al. 2018).  

After having computed the motion matrix, we apply it to the previous duckiebot rotation it to get a new estimate of the pose.

The translation vector is assumed to be always a one-component vector (pointing towards the front direction), and is scaled by a factor of the duckiebot linear velocity command.

#### Software architecture

For the development of this project, we organized the code the following way: all the code needed to handle communications using `ROS` is in the folder `ros-visualodo`. All the logic containing how the algorithms are implemented, is in `lib-visualodo`. This way we enabled an independance between `ROS` and the code itself.  

Nodes:  

visual_odometry_node:  

  * Input: The compressed camera image

  * Output: A pose estimate with respect to a world frame

  * Subsribed topics:

    - camera_node/image_compressed

    - fsm_node/switch

  * Published topics:

    - path



### Future development

The pipeline can be improved by performing bundle adjustment, however the computational burdain in that case is unclear. Other functionalities which could be interesting are a controller based on the visual odometry to travel intersections, and the placement of the path inside a given map.
