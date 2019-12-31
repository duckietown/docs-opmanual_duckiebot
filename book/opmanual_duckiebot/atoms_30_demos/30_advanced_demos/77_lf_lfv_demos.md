# LFV with Object Detection{#demo-demo-lane-following-with-vehicles status=ready}

This is a documentation for our demo which is about using Object detection to do the lane-following-with-vehicles demo.

<div class='requirements' markdown="1">

Requires: [Wheel calibration](#wheel-calibration) completed.

Requires: [Camera calibration](#camera-calib) completed.

Requires: Fully setting up Duckietown framwork and Duckiebot.

Results: One or more Duckiebot safely following lane and stops when there is a vehicle ahead.

</div>

## Video of expected results {#demo-lane-following-with-vehicles}

<!--
[link 1 of lane following with vehicles](https://drive.google.com/file/d/18o9ejgp0wOWVv8RbLE_1Ax0TUQVMIi2S/view?usp=sharing)
-->

<div figure-id="fig:demo-lane-following-with-vehicles-video">
    <figcaption>Demo: lane following with vehicles
    </figcaption>
    <dtvideo src='vimeo:247596730'/>
</div>

TODO: add a different video with an up to specification Duckietown.

## Duckietown setup notes {#demo-lane-following-with-vehicles-duckietown-setup}

To run this demo, you can setup a quite complex Duckietown. The demo supports a variety of road tiles, straight, complex turns etc. It also supports dynamic and static vehicles and is able to robustly avoid them. That makes it a level more difficult than the [lane following demo](#demo-lane-following). Make sure that your Duckietown complies with the appereance specifications presented in [the Duckietown specs](+opmanual_duckietown#dt-ops-appearance-specifications). 

You also need a wireless network setup to communicate with the robot or to `ssh` into it. The demo is robuts to varied lighting conditions but take care that the duckietown is not too dark to hinder with the object detections. 


## Duckiebot setup notes {#demo-lane-following-with-vehicles-duckiebot-setup} 
* One (or possibly more) Duckiebot in configuration [DB18](#duckiebot-configurations).
* Care must be taken that duckiebot is rigid and there are no loose parts.
* The camera must be upright so that it has a proper field of view. 
* Make sure the caliberation(both intrinsic and extrinsic is done meticulously). The performance is sensitive it.
* Make sure the battery is fixed in place.

## Pre-flight checklist {#demo-lane-following-with-vehicles-pre-flight}

* Check that every Duckiebot has sufficient battery charge and that they are all properly calibrated.
* Turn the duckiebot on and wait for it to boot. To check if it's ready try to ssh into it. 
* Place the duckiebot in the right lane. The algorithm may not allow it to recover if it is kept in the wrong ie the left lane. 
* You can use portainer to see what all containers are running on the duckiebot. In this demo we will run a new container. 

## Demo instructions {#demo-lane-following-with-vehicles-run}

### Start the demo containers

Running this demo requires almost all of the main Duckietown ROS nodes to be up and running. As these span 3 Docker images (`dt-duckiebot-interface`, `dt-car-interface`, and `dt-core`, we will need to start all of them.

Warning: First, make sure all old containers from the images `dt-duckiebot-interface`, `dt-car-interface`, and `dt-core` are stopped. These containers can have different names, instead look at the image name from which they are run.

Then, start all the drivers in `dt-duckiebot-interface`:

    laptop $ dts duckiebot demo --demo_name all_drivers --duckiebot_name ![DUCKIEBOT_NAME] --package_name duckiebot_interface --image duckietown/dt-duckiebot-interface:daffy
    
Start also the glue nodes that handle the joystick mapping and the kinematics:

    laptop $ dts duckiebot demo --demo_name all --duckiebot_name ![DUCKIEBOT_NAME] --package_name car_interface --image duckietown/dt-car-interface:daffy

Pull the docker image for LFV on the bot:

    laptop $ ssh ![DUCKIEBOT_NAME].local
    duckie@root $ docker pull charared/charan_ros_core:v1-arm32v7
    
Finally, we are ready to start the high-level pipeline for lane following with vehicles:

    laptop $ dts duckiebot demo --demo_name purepursuit_controller --duckiebot_name ![DUCKIEBOT_NAME] --package_name purepursuit --image charared/charan_ros_core:v1-arm32v7

You have to wait a while for everything to start working. While you wait, you can check in Portainer if all the containers started successfully and in their logs for any possible issues.

Portainer can be accessed at below link:

    https://![DUCKIEBOT_NAME].local:9000/#/containers

You can place your duckiebot on the lane after you see in the logs that the duckiebot is able to see the line segments. The duckiebot stops if it sees an obstacle(duckiebot) ahead and continues following the lane after the obstacle is removed.

Stopping the container demo_purepursuit_controller in the portainer will stop the lane following.


### Extras

Here are some additional things you can try:

* Get a [remote stream](#read-camera-data) of your Duckiebot.
* You can visualize the detected line segments the same way as for the [lane following demo](#demo-lane-following)
* Try to change some of the ROS parameters to see how your Duckiebot's behavior will change. 

## Troubleshooting

Symptom: The Duckiebot fails at intersections.

Resolution: This problem is an open development problem, to improve the results tune the parameters of the `unicorn_intersection_node`, the procedure is explained in the [troubleshooting section](#trouble-unicorn_intersection).

Maintainer: Contact Charan Reddy (UdeM/ Mila), Soumye Singhal (UdeM/ Mila) via Slack for further assistance.
