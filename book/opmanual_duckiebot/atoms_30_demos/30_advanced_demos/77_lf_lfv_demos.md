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

---
## Implementation details
---
### Brief Desciption of the Demo and Broad Approach.
Task is to follow the lane and avoid dynamic vehicles. 
For Lane Following we use the Pure-Pursuit Controller. It seems to work better than vanilla PID controller.
For Lane Following with Dynamic Vehicles (LFV), we consider two types of objects, static Duckies and dynamic Duckiebots. We follow a simple approach to avoid vehicles. First we do Object Detection.  We tried two methods for this 
* Supervised Learning using Faster RCNN
* HSV Thresholding based detection
Upon Detection of a nearby obstacle, in this demo we just stop the Duckiebot till the obstacle makes way or is removed. This approach can be expanded to incorporate more complex *Rule Based Maneuvering*

---

### Lane Following with the Pure-Pursuit Controller
#### Basics of Pure Pursuit Controller
We have used the purepursuit controller for the duckiebot to follow the lane. Follow point is calculated ahead in the lane using the white and yellow segments detected. 
#### Our implementation of the controller
We have used a trust segment approach, where we consider only that particular color segment if they are more in number than other in calculating follow point. If both color segments are equal in number, we take average of the centroids. Using this kind of approach avoids a jittery path of the duckiebot.

#### Dynamic velocity and omega gain for the duckiebot
We have implemented a dynamic velocity and omega gain for the duckiebot using follow point. If the y coordinate of the follow point is towards farther right or left side, it implies that there is a right or left turn there and we slow down the duckiebot and increase the omega gain to make the turn smoothly. We also vary velocity based on how farther the x coordinate of the follow point is (i.e., increase velocity if the road is straight for a long distance).


<!-- Varying Speed and Omega Gain

    Our robot detects whether it is close to a left turn, a right turn or on a straight path. Turns are detected using statistics of detected lines.
    The duckiebot gradually speeds up on straight paths, while reducing the omega gain so that the robot corrects less when moving fast (to avoid jerky movement).
    The duckiebot gradually slows down at turns, while increasing the omega gain (to make nice sharp turns).
    A second order degree polynomial is used for changing the velocity/omega gain. So, after a turn the robot speeds up slowly, giving it enough time to correct its position before going really fast. At turns, the robot will slow down faster to ensure safe navigation of the turn. -->

---
### LFV: Object Detection using **HSV Thresholding**
We have performed HSV thresholding for both simulation and the real environment. We used OpenCV trackbars to find optimal HSV values. Dilation is applied to form blobs.

#### Ground Projections

We have written our custom point2ground function to transform the detected bounding box coordinates(bottom two points of rectangle) on the ground. We have initially normalized point coordinates to original image coordinates and calculated the ground point coordinates by performing dot product of homography matrix and the point coordinates.


#### Vehicle Avoidance
<!-- 

    If we get closer to a vehicle (which is directly in front of us) than some distance threshold, we stop. We stay still until the obstalce is no longer in front of us within that distance threshold. In the visualization below, the gray box is the "safety zone" where we stop if an obstacle is within that box. -->
We use a very simple approach of just stopping on spotting an object which will collide with the current trajectory. The stopping criterion is very simple. If the object is within some deviation of our lane and withing a threshold distance then we stop, else we ignore. This threshold is fine-tuned for optimal performance. 

#### Qualitative Results in the Simulation
TODO: Add the image/gif from the slides. Also the image showing the detections. 

#### Qualitative Results in the Environment
Note that in the environment, the hsv thresholding values are different. 
TODO: Add the final video
---
### LFV: Object Detection using **Deep Learning**
This approach is mainly targetted at leveraging the available Dataset and compute to use DL to do Object Detection in Real Environment. We use logs from the real world and  Deep Learning based Faster RCNN with ResNet-50 and Feature Pyramid Network (FPN) backbone.
#### Dataset
* Dataset has images from logs of a Duckiebot in a Duckietown in a variety of lighting conditions with objects like cones, signs, duckiebots, duckies etc. 
* There are annotations for all of these objects. We only make use of the annotations for duckiebots and duckies since we assume only these in our the environment.
* Dataset as approximately 3k images
* This dataset was provided by Julian Zilly, ETH Zurich. Contact him for a copy of the datset. <a href="mailto:jzilly@ethz.ch">jzilly@ethz.ch</a>
* TODO: Preprocessing script for this dataset can be found at, 
#### Faster RCNN
* Faster RCNN is a popular object detector model that first uses a Region Proposal Network to give candidate regions based on the image statistics and for each of those regions there is a classifier for the object type(also has background class) and a regressor for the bounding boxes. 
* There are numerous tricks that are used to do efficient computation of the Feature Maps for Proposals using RoI pooling trick. 
* The backbone used in this network is the 51 layered ResNet-51.
* For more details you can refer to https://arxiv.org/abs/1506.01497
* On just using the Faster RCNN we noticed that the smaller duckies and the far away duckiebots were not detected properly. 
* So to do detection at different scales for more fine-grained detections, we use the Feature Pyramid Network. For more details look at https://arxiv.org/abs/1612.03144 
* We used the implementation by Facebook AI research in their Detectron 2 framwork. 
<p align="center">
  <img src="https://github.com/charan223/duckietown-report/blob/master/images/rcnn.png" alt="RCNN" style="width:20%">
  <img src="https://github.com/charan223/duckietown-report/blob/master/images/fpn.png" alt="FPN" style="width:20%">
</p>

Training Details
* 80:20 Train:Val Split.
* Approx 500 Duckiebot instances and 5k duckie instances.
* Pre-trained weights from MS-COCO

Exptected results
* Mean Average Precision : Duckie : 52.180  and  Duckiebot : 52.011
* Inference Time : 0.05 sec/img
* Robust to Different Lighting conditions
  
We present some qualitative results which clearly show that this method performs really well and is able to accurately detect all the duckiebots and all the ducks which are important. 

<p align="center">
  <img src="https://github.com/charan223/duckietown-report/blob/master/images/1.png" alt="det1" style="width:20%">
  <img src="https://github.com/charan223/duckietown-report/blob/master/images/2.png" alt="det2" style="width:20%">
</p>
<p align="center">
  <img src="https://github.com/charan223/duckietown-report/blob/master/images/3.png" alt="det3" style="width:20%">
  <img src="https://github.com/charan223/duckietown-report/blob/master/images/4.png" alt="det4" style="width:20%">
</p>
<p align="center">
  <img src="https://github.com/charan223/duckietown-report/blob/master/images/5.png" alt="det5" style="width:20%">
  <img src="https://github.com/charan223/duckietown-report/blob/master/images/6.png" alt="det6" style="width:20%">
</p>
<p align="center">
  <img src="https://github.com/charan223/duckietown-report/blob/master/images/7.png" alt="det7" style="width:20%">
  <img src="https://github.com/charan223/duckietown-report/blob/master/images/8.png" alt="det8" style="width:20%">
</p>

----

Initial plan to was to use this Deep Learning based classifier in the real environment using a GPU. But the process of setting up a the Duckietown framework such that the inference is done on the remote and the detections are communicated via the network is non-trivial and hence it was left for future work. We attempted to run our code on CPU but the inference times ranged between 5 to 10 seconds. So it wasn't feasible to run it on the raspberry pi for which the inference timing would have been even longer.

----

### References
* Mask R-CNN, Kaiming He, Georgia Gkioxari, Piotr Dollár, Ross Girshick, https://arxiv.org/abs/1703.06870
* Feature Pyramid Networks for Object Detection, Tsung-Yi Lin, Piotr Dollár, Ross Girshick, Kaiming He, Bharath Hariharan, Serge Belongie
* https://github.com/facebookresearch/detectron2


----
## Troubleshooting

Symptom: The Duckiebot fails at intersections.

Resolution: This problem is an open development problem, to improve the results tune the parameters of the `unicorn_intersection_node`, the procedure is explained in the [troubleshooting section](#trouble-unicorn_intersection).

Maintainer: Contact Charan Reddy (UdeM/ Mila), Soumye Singhal (UdeM/ Mila) via Slack for further assistance.
