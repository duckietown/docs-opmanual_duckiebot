# AMOD18 cSLAM {#demo-cslam status=draft}

This is the description of cSLAM.

<div class='requirements' markdown="1">

Requires: Watchtowers, Tiles, April tags (tag family=Tag36h11, size=6.5cm, border=1), Duckiebots

Requires: Wheels calibration completed.[wheel calibration](#wheel-calibration)

Requires: Camera calibration completed.[Camera calibration](#camera-calib)

Requires: Joystick demo has been successfully launched.[Joystick demo](#rc-control))

Requires: ROS installation on local computer

Requires: The duckiebot in configuration DB18 section B-11

</div>

## Video of expected results {#demo-cslam-expected}

First, we show a video of the expected behavior (if the demo is succesful).

## Duckietown setup notes {#demo-cslam-duckietown-setup}

Layout of Duckietown:

* Layout 
  - tiles shold be arranged where all path will form a closed loop
  - traffic lights are good to have but not necessary (optional)
  - Any 2 watchtower should observe at least one common April tag and all the April tags should be observed by all the watchtowers
* Instrastructure 
  - April tags have to be placed on the tiles and traffic signs.
  - For a detailed map to be visualized, the poses of the April tags in the Duckietown must be known beforehand (optional)
  - Watchtowers have to be spread across the entire Duckietown. Prefeably the combine field of view covers the entire Duckietown
* Weather
  - Lighting have to be good for the April tags to be seen clearly by camera
  - It is always sunny and rain never occurs in Duckietown

## Duckiebot setup notes {#demo-cslam-duckiebot-setup}

* All Duckiebots are required to have an April tag mounted at the top of the Duckiebot. The bottom of the April tag have to be pointing towards the rear of the Duckiebot. The center of the April tag should be aligned as closly as possible to the CG of the Duckiebot 


## Pre-flight checklist {#demo-cslam-pre-flight}

Check: the duckiebot has sufficient battery

Check: ros-picam is turned on 

Check: Joystick is turned on.

Check: ROS is installed on your local computer

## Demo instructions {#demo-cslam-run}

### Step 0: Before starting, please install ROS on your local computer by following the official installation instructions [here](http://wiki.ros.org/kinetic/Installation/Ubuntu). Please install the Desktop-Full  version.

### Step 1: Set up the Watchtowers
    (Provide link to set up watchtowers. Building and software initialization)

### Step 2: Print out the April tags and place them on top of Duckiebots and in Duckietown
    (Provide location for Benson to place pre-generated April tags)

### Step 3: Pull the Acquisition, Diagnostics, Simulator and Graph pose optimizer Docker images onto your laptop:

    laptop $ (fill up)
    
### Step 4: Run the pulled Docker images:

    laptop $ (fill up)

### Step 5: Control the Duckiebot manually around Duckietown

    laptop $ dts duckiebot keyboard_control ![hostname]


## Troubleshooting {#demo-cslam-troubleshooting}

### April tags printed may be of wrong size
Check that the print April tags are of size 6.5cm as the printer might have done some scaling to the tags.

## Demo failure demonstration {#demo-cslam-failure}

Finally, put here a video of how the demo can fail, when the assumptions are not respected.
