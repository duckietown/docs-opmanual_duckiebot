# LFV with Object Detection {#lane-following-with-vehicles status=ready}

This is a documentation for our demo, which is about using Object detection to do the lane-following-with-vehicles demo.

<div class='requirements' markdown="1">

Requires: Wheel and Camera calibration.

Requires: Fully setting up Duckietown framework and Duckiebot.

Results: One or more Duckiebot safely following lane and stops when there is a vehicle ahead.
</div>

## Video of expected results {#demo-lane-following-with-vehicles-expected}

[Video Link (Real) - Lane following with vehicles](https://drive.google.com/open?id=18o9ejgp0wOWVv8RbLE_1Ax0TUQVMIi2S)

## Duckietown setup notes {#demo-lane-following-with-vehicles-duckietown-setup}

To run this demo, you can setup a quite complex Duckietown. The demo supports a variety of road tiles, straight, complex turns, etc. It also supports dynamic and static vehicles and can robustly avoid them. That makes it a level more difficult than the lane following demo.  

You also need a wireless network set up to communicate with the robot or to `ssh` into it. The demo is robust to varied lighting conditions but take care that the duckietown is not too dark to hinder with the object detections. 

* Add one (or possibly more) Duckiebot in duckiebot configurations.
* Care must be taken that duckiebot is rigid and there are no loose parts.
* The camera must be upright so that it has a proper field of view. 
* Make sure the calibration(both intrinsic and extrinsic is done meticulously). The performance is sensitive to it.
* Make sure the battery is fixed in place and fully charged.

## Pre-flight checklist {#demo-lane-following-with-vehicles-pre-flight}

* Check that every Duckiebot has sufficient battery charge and that they are all properly calibrated.
* Turn the duckiebot on and wait for it to boot. To check if it's ready, try to ssh into it. 
* Place the duckiebot in the right lane. The algorithm may not allow it to recover if it is kept in the wrong ie, the left lane. 
* You can use portainer to see what all containers are running on the duckiebot. In this demo, we will run a new container. 


## Demo instructions {#demo-lane-following-with-vehicles-run}

### [Github Link to the Package for the Demo](https://github.com/charan223/charan_ros_core)

### Setup

Fork and clone udem-fall19-public:

    laptop $ git clone https://github.com/charan223/udem-fall19-public.git

If you are using fork, create an upstream:

    laptop $ git remote add upstream https://github.com/duckietown-udem/udem-fall19-public.git

Then pull from upstream:

    laptop $ git pull upstream master 


Update submodules

    laptop $ git submodule init
    laptop $ git submodule update
    laptop $ git submodule foreach "(git checkout daffy; git pull)"

### To run in simulation


From udem-fall19-public directory run:

    laptop $ docker-compose build
    laptop $ docker-compose up

You can open the notebook just like before by copying the url that looks like:

    http://127.0.0.1:8888/?token={SOME_LONG_TOKEN}

After that, open terminal in the notebook and run below commands:

    jupyter-notebook $ catkin build --workspace catkin_ws
    jupyter-notebook $ source catkin_ws/devel/setup.bash
    jupyter-notebook $ ./launch_car_interface.sh

Run your launch file using below commands:

    jupyter-notebook $ roslaunch purepursuit purepursuit_controller.launch

Visualize it in the simulation at `http://localhost:6901/vnc.html` in your browser, with `quackquack` as the password. Run `rqt_image_view` in the terminal to visualize the image topics.

### To run on real bot

Go to package's base directory and do

    laptop $ dts devel build --push

Pull the docker image for LFV on the duckiebot:

    laptop $ ssh ![DUCKIEBOT_NAME].local
    duckie@root $ docker pull charared/charan_ros_core:v1-arm32v7
 
To run LFV on the duckiebot:

    laptop $ dts duckiebot demo --demo_name purepursuit_controller --duckiebot_name ![DUCKIEBOT_NAME] --package_name purepursuit --image charared/charan_ros_core:v1-arm32v7


You have to wait a while for everything to start working. While you wait, you can check in Portainer if all the containers started successfully and in their logs for any possible issues.

Portainer can be accessed at below link:

    https://![DUCKIEBOT_NAME].local:9000/#/containers

You can place your duckiebot on the lane after you see in the logs that the duckiebot can see the line segments. The duckiebot stops if it sees an obstacle(duckiebot) ahead and continues following the lane after the obstacle is removed.

Stopping the container demo_purepursuit_controller in the portainer will stop the lane following.


<!--
### Extras

Here are some additional things you can try:

* Get a [remote stream](#read-camera-data) of your Duckiebot.
* You can visualize the detected line segments the same way as for the [lane following demo](#demo-lane-following)
* Try to change some of the ROS parameters to see how your Duckiebot's behavior will change. 
-->

---

## Implementation details

---

### Description of the Demo and the Approach.
The task is to follow the lane and avoid dynamic vehicles. 
For Lane Following, we use the Pure-Pursuit Controller. It seems to work better than the vanilla PID controller.
For Lane Following with Dynamic Vehicles (LFV), we consider two types of objects, static Duckies, and dynamic Duckiebots. We follow a simple approach to avoid vehicles. First, we do Object Detection. We tried two methods for this 
* Supervised Learning using Faster RCNN
* HSV Thresholding based detection
Upon Detection of a nearby obstacle, in this demo, we just stop the Duckiebot till the obstacle makes way or is removed. This approach can be expanded to incorporate more complex *Rule-Based Maneuvering*

---

### Lane Following with the Pure-Pursuit Controller
#### Basics of Pure Pursuit Controller
We have used the purepursuit controller for the duckiebot to follow the lane. Follow point is calculated ahead in the lane using the white and yellow segments detected. 
#### Our implementation of the controller
We have used a trust segment approach, where we consider only that particular color segment if they are more in number than others in calculating the follow point. If both color segments are equal in number, we take an average of the centroids. Using this kind of approach avoids a jittery path of the duckiebot.

#### Dynamic velocity and omega gain for the duckiebot
We have implemented a dynamic velocity and omega gain for the duckiebot using follow point. If the y coordinate of the follow point is towards the farther right or left side, it implies that there is a right or left turn there, and we slow down the duckiebot and increase the omega gain to make the turn smoothly. We also vary velocity based on how farther the x coordinate of the follow point is (i.e., increase velocity if the road is straight for a long distance).


---
### LFV: Object Detection using **HSV Thresholding**
The most basic way to detect objects which can work in both simulation and the real environment is doing HSV thresholding and then doing opencv operations to get the bounding box for the object. A brief description is as follows
* We have performed HSV thresholding for both simulation and the real environment separately
* The `pure_pursuit` node on receiving an image processes it and determines if an object is there. 
* HSV thresholding for reds and yellows is done to nicely detect the duckiebots and duckies, to form a mask. We used OpenCV trackbars to find optimal HSV values.
* Dilation is applied to form good blobs. 
* Then we find contours using `cv2.findContour` to aggregate the blobs. We use those contours to get a nice bounding box around the detection
 
Now after the object is detected, we need to compute how far it is from our duckiebot and whether the stopping criterion is satisfied. This is described next:

#### Ground Projections

We have written our custom `point2ground` function to transform the detected bounding box coordinates(bottom two points of the rectangle) on the ground. We have initially normalized point coordinates to original image coordinates and calculated the ground point coordinates by performing dot product of homography matrix and the point coordinates. Now the projected points tell us how far the detected object is from our robot.


#### Vehicle Avoidance
We use a very simple approach of just stopping on spotting an object which will collide with the current trajectory. The stopping criterion is very simple. If the object is within some deviation of our lane and within a threshold distance, then we stop, else we ignore. This distance is obtained by using the ground projection node described before. This threshold is fine-tuned for optimal performance. 

#### Qualitative Results in the Simulation
The following is the results obtained from doing hsv thresholding in the simulation. We can see that it does a good job of detecting all the objects.

<p align="center">
 <img src="https://github.com/charan223/duckietown-report/raw/master/gifs/hsv.gif" alt="hsv gif" width="400" height="300"/>
</p>

Qualitatively we can also see the detection and the consequent stopping of the duckiebot after the thresholding in simulation.

<p align="center">
 <img src="https://github.com/charan223/duckietown-report/raw/master/gifs/hsv1.gif" alt="hsv1 gif" width="400" height="300"/>
</p>


[Full Video Link (Simulation) - Lane following with vehicles](https://drive.google.com/open?id=1-oaqhY2mspkT7VWq6Dqx_lCsA4yOZF5w)

#### Qualitative Results in the Environment
Note that in the environment, the HSV thresholding values are different. 

<p align="center">
 <img src="https://github.com/charan223/duckietown-report/raw/master/gifs/lfv_cut11.gif" alt="lfv_cut11 gif" width="400" height="300"/>

 <img src="https://github.com/charan223/duckietown-report/raw/master/gifs/lfv_cut22.gif" alt="lfv_cut22 gif" width="400" height="300"/>
</p>

[Full Video Link (Real) - Lane following with vehicles](https://drive.google.com/open?id=18o9ejgp0wOWVv8RbLE_1Ax0TUQVMIi2S)

[Full Video Link (Real) - Lane following](https://drive.google.com/open?id=18vinMYckb0UH0hNQebdYQNFNohcRcu9Z)


---

### LFV: Object Detection using Deep Learning
This approach is mainly targetted at leveraging the available Dataset and compute to use DL to do Object Detection in Real Environment. We use logs from the real world, and Deep Learning-based Faster RCNN with ResNet-50 and Feature Pyramid Network (FPN) backbone.
#### Dataset
* Dataset has images from logs of a Duckiebot in a Duckietown in a variety of lighting conditions with objects like cones, signs, duckiebots, duckies etc. 
* There are annotations for all of these objects. We only make use of the annotations for duckiebots and duckies since we assume only these in our environment.
* Dataset as approximately 3k images
* This dataset was provided by Julian Zilly, ETH Zurich. Contact him for a copy of the dataset. <a href="mailto:jzilly@ethz.ch">jzilly@ethz.ch</a>
* TODO: Preprocessing script for this dataset can be found at, 
#### Faster RCNN
* Faster RCNN is a popular object detector model that first uses a Region Proposal Network to give candidate regions based on the image statistics, and for each of those regions, there is a classifier for the object type(also has background class) and a regressor for the bounding boxes. 
* There are numerous tricks that are used to do efficient computation of the Feature Maps for Proposals using RoI pooling trick. 
* The backbone used in this network is the 51 layered ResNet-51.
* For more details you can refer to https://arxiv.org/abs/1506.01497
* On just using the Faster RCNN, we noticed that the smaller duckies and the far away duckiebots were not detected properly. 
* So to do detection at different scales for more fine-grained detections, we use the Feature Pyramid Network. For more details look at https://arxiv.org/abs/1612.03144 
* We used the implementation by Facebook AI research in their Detectron 2 framework. 
* Link to our implementation here: [Colab Link](https://colab.research.google.com/drive/1lUMuVuZzaiQDyc_2fBSUKJusULbzJgvj)
<p align="center">
 <img src="https://github.com/charan223/duckietown-report/raw/master/images/rcnn.png" alt="RCNN" width="400" height="500"/>
 <img src="https://github.com/charan223/duckietown-report/raw/master/images/fpn.png" alt="FPN" width="400" height="250"/>
</p>

Training Details
* We used an 80:20 Train, Val Split.
* Approx 500 Duckiebot instances and 5k duckie instances were used. 
* For the backbone we use pre-trained weights from MS-COCO. So we need to only fine-tune to our dataset. Which is time-efficient as well as allows us to train on our small-dataset.

Obtained Results
* Mean Average Precision(MAP) for duckies was 52.180 and for duckiebot was 52.011
* Inference Time for the trained model is 0.05 sec/img
* The detections are robust to different lighting conditions, as can be seen from the results below.
 
We present some qualitative results which clearly show that this method performs really well and is able to accurately detect all the duckiebots and all the ducks which are important. 

<p align="center">
 <img src="https://github.com/charan223/duckietown-report/raw/master/images/1.png" alt="det1" width="270" height="230"/>
 <img src="https://github.com/charan223/duckietown-report/raw/master/images/2.png" alt="det2" width="270" height="230"/> 
 <img src="https://github.com/charan223/duckietown-report/raw/master/images/3.png" alt="det3" width="270" height="230"/> 
</p>
<p align="center">
 <img src="https://github.com/charan223/duckietown-report/raw/master/images/5.png" alt="det4" width="270" height="230"/>
 <img src="https://github.com/charan223/duckietown-report/raw/master/images/6.png" alt="det5" width="270" height="230"/>
 <img src="https://github.com/charan223/duckietown-report/raw/master/images/7.png" alt="det6" width="270" height="230"/>
</p>
 
----

The initial plan was to use this Deep Learning-based classifier in the real environment using a GPU. But the process of setting up the Duckietown framework such that the inference is done on the remote and the detections are communicated via the network is non-trivial, and hence it was left for future work. We attempted to run our code on CPU, but the inference times ranged between 5 to 10 seconds. So it wasn't feasible to run it on the raspberry pi for which the inference timing would have been even longer.


---
## Performance

**RUNNER UP** during LFV Challenge at NeurIPS, Vancouver 2019

**RUNNER UP** during LF Challenge at UdeM, Montreal 2019
### AI Driving Olympics Leaderboard
| Challenge | Position |
|----------------------|:----------:|
| aido3-LFV-real-validation | 2 |
| aido3-LFV-sim-testing | 6 |
| aido3-LFV-sim-validation | 8 |
| aido3-LF-real-validation | 8 |
| aido3-LF-sim-testing | 10 |
| aido3-LF-sim-validation | 10 |

----

### References
* Mask R-CNN, Kaiming He, Georgia Gkioxari, Piotr Dollár, Ross Girshick, https://arxiv.org/abs/1703.06870
* Feature Pyramid Networks for Object Detection, Tsung-Yi Lin, Piotr Dollár, Ross Girshick, Kaiming He, Bharath Hariharan, Serge Belongie
* https://github.com/facebookresearch/detectron2


----

Maintainer: Contact Charan Reddy (UdeM/ Mila), Soumye Singhal (UdeM/ Mila), via Slack for further assistance.


