# AMOD18 Project Unicorn {#demo-projectunicorn status=beta}

**Project Unicorn** is a project for AMOD (Autonomous Mobility On Demand) course from ETH Zürich that focuses on the intersection navigation for duckiebots in Duckietown.

The following document provides the instructions on how to run the intersection navigation demo on a duckiebot and a basic overview of the project (in the [section below](#demo-projectunicorn-description)).

The interested reader can find the source code in the [Project Unicorn Intersection Navigation repository](https://github.com/duckietown/duckietown-intnav).

<div class='requirements' markdown="1">


**Requires**: Duckiebot in [DB18 configuration](#duckiebot-configurations)


**Requires**: Completed [camera calibration](#camera-calib)


**Requires**: Completed [wheel calibration](#wheel-calibration)


**Requires**: Local computer with [Docker installation](#preliminaries-docker-basics)


</div>


### Demo workflow {#demo-projectunicorn-workflow}

**Intersection Navigation demo:**

The Intersection Navigation demo performs one intersection maneuver out of the following 3:

* Left

* Right

* Straight

For each intersection type (3-way or 4-way) a feasible direction will be chosen randomly by the algorithm.

**Intersection Navigation and Lane following demo:**

Additionally to the official demo (“pure” intersection navigation), a demo combining lane following and intersection navigation can be executed.

In this case, the duckiebot will perform one intersection maneuver and switch to lane following automatically after.

To avoid unnecessary computation while performing intersection navigation in our lane following image, the camera publishes on different topics: one for lane following and the other one for intersection navigation.

Note: As a red line detection module was not operational during our project, the lane follower will not stop at the next intersection but continue lane following.

## Video of expected results {#demo-projectunicorn-expected}

The expected behaviors should look like the following videos:

<div figure-id="fig:success_unicorn">
    <figcaption>Expected behavior of the duckiebot when performing the intersection navigation demo (3 different demos for each of the directions).
    </figcaption>
    <dtvideo src='vimeo:307407118'/>
</div>


<div figure-id="fig:success_unicorn2">
    <figcaption>Expected behavior of the duckiebot when performing the intersection navigation and lane following demo.
    </figcaption>
    <dtvideo src='vimeo:308420740'/>
</div>

## Duckietown setup notes {#demo-projectunicorn-duckietown-setup}

The following is assumed:

* Standard 3 and 4-way intersections in size and orientation; built according to Duckietown specifications.

* 2 AprilTags are placed facing each of the different stopping lines according to [](#fig:4-way) and [](#fig:3-way).

<div figure-id="fig:4-way" figure-caption="Correct position of the Apriltags in a 4-way Intersection" >
     <img src="4-way-map.png" style='width: 30em'/>
</div>

<div figure-id="fig:3-way" figure-caption="Correct position of the Apriltags in a 3-way Intersection" >
     <img src="3-way-map.png" style='width: 30em'/>
</div>


## Laptop setup notes {#demo-projectunicorn-laptop-setup}

Clone the duckietown-intnav folder in your PC:

    laptop $ git clone --branch demo git@github.com:duckietown/duckietown-intnav.git

## Duckiebot setup notes {#demo-projectunicorn-duckiebot-setup}

**Requires**: Completed intersection navigation calibration.

An accurate localization in the intersection area is crucial for successful navigation.

The used algorithm is inter alia based on reprojecting points from the camera to the world frame. Therefore in order to enable an accurate localization an accurate camera calibration is required, especially with respect to scale.

**Calibration test instructions:**

###st Step

Place the duckiebot in the intersection as in [](#fig:test_position): the duckiebot's origin (center of wheel axis) in the center of the bottom line of the yellow tag adjacent to the red stop line and oriented towards the opposite side of the intersection.

<div figure-id="fig:test_position" figure-caption="Correct duckiebot position for the intersection calibration test" >
     <img src="test_position.jpg" style='width: 30em'/>
</div>

###nd Step

Run the container on your duckiebot:

    laptop $ cd duckietown-intnav/scripts
    laptop $ bash deploy.bash ![hostname]

###rd Step

Start the calibration test in the intnav container:

    duckiebot $ roslaunch duckietown-intnav calibration_check.launch duckiebot:=![hostname]

###th Step

When launched, press <kbd>X</kbd> to start the calibration procedure.

### Test results:

Better: Test passed: no further actions.

Bad: Test failed: the camera has to be [recalibrated](#camera-calib), with special regards on the scale.

Note: To obtain a good camera calibration in terms of scale, make sure the bar checking for "Scale" is filled above 70% (like in [](#fig:camera_calibration)).

Recommended: place the camera very close to the checkerboard during the data collection for the calibration process.

<div figure-id="fig:camera_calibration" figure-caption="Acceptable camera calibration example" >
     <img src="camera_calibration.png" style='width: 30em'/>
</div>


## Pre-flight checklist {#demo-projectunicorn-pre-flight}

**Check**: The duckiebot has sufficient battery.

**Check**: The intersection is free of obstacles (including other duckiebots).

## Intersection Navigation demo instructions {#demo-projectunicorn-run}

###st Step

Place the duckiebot in front of any of the red lines from the desired Duckietown intersection.

###nd Step

Run the container on your duckiebot:

    laptop $ cd duckietown-intnav/scripts
    laptop $ bash deploy.bash ![hostname]

###rd Step

Once inside the container, start the intnav demo:

    duckiebot $ roslaunch duckietown-intnav main.launch duckiebot:=![hostname]

###th Step

After the duckiebot stops, the demo is finished.

Next steps: Go back to Step 1 if you wish to navigate the intersection again.


## Additional demo: Intersection Navigation and Lane following demo instructions {#demo-projectunicorn-demo2}

###st Step

Run the container on your duckiebot:

    laptop $ cd duckietown-intnav
    laptop $ git checkout master
    laptop $ cd scripts
    laptop $ bash deploy_lane_follower.bash [hostname]

###nd Step

Wait until the lane follower is booted; it can take a few minutes. Place your duckiebot at an intersection.

###rd Step

Start intersection navigation. In another terminal:

    laptop $ bash deploy.bash [hostname]
    duckiebot $ roslaunch duckietown-intnav main.launch [hostname]

The duckiebot will cross the intersection and switch to lane following automatically.

Note: the duckiebot is not expected to stop at the next red line. (Read [this](#demo-projectunicorn-workflow) for more information).

## Troubleshooting {#demo-projectunicorn-troubleshooting}

Symptom: roslaunch not working properly.

Resolution:

Stop the process with <kbd>Ctrl</kbd> + <kbd>C</kbd>.

Exit the container:

    duckiebot $ exit

Restart the container from the terminal (go back to [Step 2](#demo-projectunicorn-run)).


Symptom: Duckiebot not moving.

Resolution: Make sure the duckiebot can see AprilTags.


Symptom: Duckiebot is not navigating the intersection properly (not moving smoothly or cutting corners).

Resolution: Make sure the [wheel calibration](#wheel-calibration) is done correctly.

Resolution: Make sure the AprilTags are placed according to [](#fig:4-way) and [](#fig:3-way).

Symptom: 'roslaunch xml error' displayed.

Resolution: Try to restart the container again (try 2-3 times). If the error is not fixed, re-flash your SD card.

Symptom: Docker failed to register layer: 'no space left on device'

Resolution: Remove unused images on Portainer.

Note: Check for the newest updates on error troubleshooting on our [repository](https://github.com/duckietown/duckietown-intnav).

## Demo failure demonstration {#demo-projectunicorn-failure}

The following video shows how the Intersection Navigation demo can fail, when the assumptions are not respected.

<div figure-id="fig:fail_unicorn">
    <figcaption>When the AprilTags are not correctly placed, the duckiebot is not able to localize properly and the demo fails.
    </figcaption>
    <dtvideo src='vimeo:307407768'/>
</div>

## Project description {#demo-projectunicorn-description}

### Mission

Navigate any duckietown standard intersection from any starting point without detaining other duckietown operations at a success rate greater than 90%.

### Motivation

Intersections are an essential part of duckietowns, since without them, duckiebots can only drive autonomously on straight roads or loops. However, no method was found yet, that allows duckiebots to navigate intersections quick and reliably. Since this is a core functionality of every autonomous vehicle that interacts with street-like infrastructure, the approach implemented in this project tackles this issue.

### Problem statement

We must be able to drive duckiebots through any of the intersections. The implementation should be such that the duckiebot is not crossing any lines during the intersection navigation. In order to drive the robot to any exit point, for all initial conditions on distance and orientations at the entrance point, within a certain TBD bound.

### Existing Solutions

**Standard Implementation:**

The intersection navigation that is currently used for demos is an extended version of the duckietown lane follower. While the duckietown lane follower is robust enough to let the duckiebot follow a line without failing until its battery is empty, the extended version for intersection navigation not working reliably.

**2017 Navigators team implementation:**

The Navigators team implemented a template matching based approach, based on the paper [Edge-Based Markerless 3D Tracking of Rigid Objects.](https://ieeexplore.ieee.org/document/4414647)
Due to the computational heaviness of the approach, a fast navigation was not achieved, and due to slow control update cycles, the success rate of the standard implementation could not be improved.

### Our Approach {#demo-projectunicorn-approach}

As indicated in grey in [](#fig:software) the intersection navigation goes through four main steps:

<div figure-id="fig:software" figure-caption="Diagram showing the approach" >
     <img src="Diagram_Unicorn.png" style='width: 35em'/>
</div>

**1- Estimation of the initial pose**: Starting at a red line at any intersection, the duckiebot estimates its initial pose. The duckiebot relative position in the intersection with regard to the AprilTags (based on camera image) is calculated ([AprilTag 2: Efficient and robust fiducial detection](https://april.eecs.umich.edu/pdfs/wang2016iros.pdf)) and added to the fixed AprilTags relative position with regard to the global frame to obtain the duckiebot initial pose in the global frame.

**2- Trajectory Generation**: different paths for going left, right and straight are pre-computed and chosen depending on the desired intersection exit. Given the intersection type, the intersection command (which can be given as user input or as a random chose) and bearing in mind the duckiebot dynamic constraints and duckietown intersection boundaries, the proper trajectory is generated.

**3- Pure Pursuit Controller**: Constantly updating its pose estimate, it follows the path in a closed-loop manner thanks to the PurePursuit path tracking algorithm from the controller. Implementation based on the paper [Automatic Steering Methods for Autonomous Automobile Path Tracking.](https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf)

**4- Interface**: detects when the duckiebot has reached the intersection end (exit lane) and switches back to the duckietown lane follower.

### Assumptions

Besides the requirements stated in the requirements, [setup notes](#demo-projectunicorn-duckietown-setup) and [pre-flight list](#demo-projectunicorn-pre-flight), the following assumptions are made:

**Hardware:**

* Differential drive robot (DB18).

* Camera with 160º field of view.

**Software:**

The duckiebot is placed in a lane (with width between 10 - 16 cm), in front of the red line and  approximately perpendicular to it.

* Duckiebot drives successfully on lines: reliable lane following mode.

* Initial intersection conditions:

    - Duckiebot detecting intersection.

    - Stopping in front of the red line: Initial position within the lane and within ± 3 cm to the red line and initial orientation within ± 10° to the yellow lines.

    - Entering intersection navigation mode.

* Maximal control delay (~ 1 s, testable).

### Visualization

To watchdog the performance of the algorithm a rviz interface is available as well as display of the detected april tag.

    laptop $ bash run_visualization.bash ![hostname]

<div figure-id="fig:visualization" figure-caption="AprilTag detection" >
     <img src="visualization.jpg" style='width: 30em'/>
</div>

### Failed Approaches

For the [Estimation of the initial pose](#demo-projectunicorn-approach), the following approaches other than the AprilTag detection were tried and not implemented for the reasons stated below:

**Feature Detection + Template matching:**

The camera image was re-projected to a birds-eye-view image using the homography. In this reprojected image, features were detected using the FAST-detector. To relate them to known points in the intersection, a color-based descriptor was tested that was then related to known descriptors of the given points. However, this approach turned out to be computationally heavy and did not allow for robust matching between detected and known points.

**Corner Detection:**

The camera image was converted to HSV colorspace. Then, canny edge detector was used to detect lines. Different thresholds were used to discard yellow and red lines from white lines, since the latter seemed to be the easiest to detect. Since the world coordinates of intersecting white lines are known, this would provide a way to localize in the map. However, this approach includes lots of threshold tuning. While it was possible to tune these thresholds according for a single environment and camera scenario, no threshold setting was found that was robust in different lightning conditions (due to changing white-balance, reflections of lines etc).

### Software architecture

The code is split into two sections:

* **lib-intnav**: contains the logic architecture and algorithms of the demo implementation. It contains the following nodes:

<col4 figure-id="tab:mytable" figure-caption="Nodes from lib-intnav" class="labels-row1">
    <span>Node</span>
    <span>Description</span>
    <span>Input</span>
    <span>Output</span>
    <span>camera_config</span>
    <span>Undistorsion algorithm</span>
    <span>Camera image, Camera calibration</span>
    <span>Undistorsioned image</span>
    <span>kalman</span>
    <span>Extended Kalman filter based on differential drive mode</span>
    <span>Path to follow, Instante pose, Measurements of previous pose</span>
    <span>Estimated next pose</span>
    <span>planner</span>
    <span>Generate corresponding path to follow</span>
    <span>Intersection command</span>
    <span>Path to follow</span>
    <span>controller</span>
    <span>Control duckiebot locally to follow optimal trajectory</span>
    <span>Path to follow, Instant pose</span>
    <span>Wheel speed commands</span>
    <span>imap</span>
    <span>Visualize the intersection</span>
    <span>Intersection type</span>
    <span>Intersection map</span>
    <span></span>
    <span></span>
    <span></span>
</col4>

* **ros-intnav**: contains all nodes that enable the communication between lib-intnav (algorithms for calculation) and ROS (communicates with the duckiebot). It contains the following nodes:

<col4 figure-id="tab:mytable2" figure-caption="Nodes from ros-intnav" class="labels-row1">
    <span>Node</span>
    <span>Libraries (lib-intnav /external)</span>
    <span>Input</span>
    <span>Output</span>
    <span>image_ processing</span>
    <span>image_ calibration</span>
    <span>Raw image</span>
    <span>Undistorted image</span>
    <span>april_ activator</span>
    <span></span>
    <span></span>
    <span>AprilTag detection ping</span>
    <span>localization</span>
    <span>kalman</span>
    <span>Previous wheel speeds, Detected tags</span>
    <span>Pose estimation, Trajectory</span>
    <span>controller</span>
    <span>controller, planner</span>
    <span>Pose estimate</span>
    <span>Optimal path, wheel speed commands</span>
    <span>interface</span>
    <span></span>
    <span>Duckietown configuration, Pose</span>
    <span>Direction to go, Intersection type, Switch</span>
    <span>tf_april_ static</span>
    <span>tf</span>
    <span>Duckietown configuration</span>
    <span>AprilTag pose in world frame</span>
    <span>tf_cam_ vehicle</span>
    <span>tf</span>
    <span>Duckiebot configuration</span>
    <span>Camera to wheel axis transformation</span>
    <span>visualization _imap</span>
    <span>imap</span>
    <span>Direction to go, Intersection type</span>
    <span>Visualization (rviz)</span>
</col4>

### Performance analysis

* Success rate = Number of successful navigations / Number of trials = 93%

A successful intersection navigation entails:

	- Following the path in a reasonable way: not crossing any lines, smooth trajectory.

	- Achieving the desired intersection end point within a range of +/-15 cm.

	- Navigating the intersection within the maximum time limit: 10 s for straight and left curve, 8 s for right curve.

* Navigation duration (until duckiebot is able to continue with the lane follower):

Left = 8 s (3 s initialize + 5 s operation)

Right = 6 s (3 s initialize + 3 s operation)

Straight = 6 s (3 s initialize + 3 s operation)

* Repeatability: Start intersection navigation from a fixed point and compare the end points for robustness.
