# Coordination {#demo-coordination2016 status=beta}

TODO: mark this as obsolete (2016). 2017 version is out: explicit coordination, implicit coordination.

This demo (instructions from the MIT 2016 class) allows multiple Duckiebots that stop at an intersection to coordinate with LED signals and clear the intersection. Two demos are available, one for the case with traffic lights and one without them.

<div class='requirements' markdown="1">

Requires: Duckiebot in configuration DB17-lc

Requires: Camera calibration and kinematic calibration completed.

</div>


## Duckietown setup notes

The Duckietown used for these demos needs to have the following characteristics:

* Three or four way intersection tiles.
* The intersections must be provided with two signs that have to be clearly visible:

1) The intersection type (stop sign or traffic light),

2) Intersection topology. Traffic light if needed.


## Duckiebot setup notes

No special setup is needed for the Duckiebot. If more Duckiebots are available, they should be used too since the demo is about coordination and LED emission and detection. One to four Duckiebots can be placed at an intersection.


## Demo instructions {#demo-coordination-run}

### Openhouse-dp4 (traffic light coordination)

This demo is for the Traffic-light coordination. The intersection must therefore be provided with a traffic light.

Everything should be run from branch: phase-3-integration

* Step 1: Run the following commands:

Make sure you are in the Duckietown folder:

    $ cd ~/duckietown

Activate ROS:

    $ source environment.sh

On the traffic light run:

    $ make traffic-light veh:=$traffic_light_name

On the Duckiebot run:

    duckiebot &#36; make openhouse-dp4

* Step 2: Wait for build to finish. Press 'X' to run anti-instagram. After you see the message ‘transform
has been published’, place the Duckiebot on the lane and press 'R1'. To switch to manual mode press 'L1'

#### Expected outcomes

* The Duckiebot should stop at traffic light intersections, and wait for the green light to flash (with the right frequency).

* When the green light comes on, it will take the Duckiebot a few seconds to react, before proceeding across the intersection.

* Currently the direction of turn is randomized, so the Duckiebot should choose a random turning direction.

### Openhouse-dp5 (stop sign explicit coordination)

This demo is for the stop sign coordination. The intersection is without a traffic light

Everything should be run from branch: phase-3-integration

* Step 1: Run the following commands:

Make sure you are in the duckietown folder:

    $ cd ~/duckietown

Activate ROS:

    $ source environment.sh

On the Duckiebot run:

    duckiebot &#36; make openhouse-dp5

* Step 2: Wait for build to finish. Press 'X' to run anti-instagram. After you see the message ‘transform
has been published’, place the Duckiebot on the lane and press 'R1'. To switch to manual mode press 'L1'

####Expected outcomes

* The Duckiebot should stop at the intersection and 'look around' for other Duckiebots at the intersection (on the right).

* After other robots at the intersection clears (i.e. finished crossing, others gave 'pass' signal), the Duckiebot should start flashing 'I am going' signal and crosses the intersection.

## Troubleshooting 

* When it shows "Event:intersection_go", the Duckiebot does not move. This problem is related to AprilTags.

Solution: go to config/baseline/pi_camera and change the framerate to 30 instead of 15.

* [Warning] Topics '/![robot name]/camera_node/image/rect' and '/![robot name]/camera_node/raw_camera_info' do not appear to be synchronized.
Solution:
