# Explicit Coordination {#demo-coordination2017 status=beta}

This demo allows different Duckiebots to coordinate at an intersection using LED-based communication. It handles three or four way intersections with or without a traffic light.


<div class='requirements' markdown="1">

Requires: Duckiebot in configuration DB17-lc

Requires: Camera calibration and kinematic calibration completed.

</div>


## Duckietown setup notes

The Duckietown used for this demo needs to have the following characteristics:

* Three or four way intersection tiles.
* The intersections must be provided with two signs that have to be clearly visible:

  1) The intersection type (stop sign or traffic light),

  2) Intersection topology. Traffic light if needed.

## Duckiebot setup notes

No special setup is needed for the Duckiebot. If more Duckiebots are available, they should be used too since the demo is about coordination and LED emission and detection. One to four Duckiebots can be placed at an intersection.


## Demo instructions {#demo-explicit-instructions}

### If there is a traffic light:

To start the traffic light:

* Make sure the traffic light is connected to power

* From your laptop connect to wifi TrafficLight# (# corresponds to a decimal number), the password is 'Traff!cL!ght'

SSH into the raspberry pi:

    laptop $ ssh tlo@trafficlight#.local

The password is 'Traff!cL!ght'

* To start the demo on the traffic light:

Make sure you are on the branch new_light_traffic:

    $ cd ~/duckietown

    $ git fetch --all

    $ git checkout new_light_traffic

Start the demo:

    $ make traffic-light



### On the Duckiebot:

Make sure you are in the Duckietown folder:

    duckiebot &#36; cd ~/duckietown

Checkout to branch devel-explicit-coord-jan15:

    duckiebot &#36; git fetch --all

    duckiebot &#36; git checkout devel-explicit-coord-jan15

Run the demo:

    duckiebot &#36; make coordination2017


Wait for build to finish. Place the Duckiebot on the lane and press 'R1'. To switch to manual mode press 'L1'

#### Expected outcomes


* The Duckiebot should stop at traffic light intersections, and wait for the green light to flash.

* The Duckiebot should stop at red stop line and coordinate with other Duckiebots that are also stopped at the same intersection, if there are some, according to the communication protocol and wait until it has assessed that it can proceed to navigate the intersection.

* When the green light comes on, or the Duckiebot has decided that is time to go, it will start to clear the intersection.

* Currently the direction of turn is randomized, so the Duckiebot should choose a random turning direction.



## Troubleshooting {#demo-explicit-troubleshooting}

* When it shows "Event:intersection_go", the Duckiebot does not move. This problem is related to AprilTags.

Solution: go to config/baseline/pi_camera and change the framerate to 30 instead of 15.

* [Warning] Topics '/![robot name]/camera_node/image/rect' and '/![robot name]/camera_node/raw_camera_info' do not appear to be synchronized.
Solution:
