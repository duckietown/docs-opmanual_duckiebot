# Demo unsupervised image segmentation {#demo-kmeans status=beta}

This demo shows implemenation of k-means algorithm on the duckiebot for image segmentation.

In order to complete the procedure, you need your duckiebot in configuration DB18 until unit B-11 with its camera calibrated.

<div class='requirements' markdown="1">

Requires: The duckiebot in configuration DB18 section B-11

Requires: Camera calibration completed.

</div>

## Pre-flight checklist {#demo-kmeans-pre-flight}

Check: the duckiebot has sufficient battery

Check: the camera is calibrated


## Demo instructions {#demo-kmeans-run}

**Step 1**: Now we need to run the docker container to be able to launch the camera_kmeans node on our duckiebot. Check if DOCKER_HOST variable is set

    laptop $ echo $DOCKER_HOST

in case it is not, then set it

    laptop $ export DOCKER_HOST=![ROBOT_NAME].local

Now, run the docker container `kmeans` on your duckiebot

    laptop $ docker -H ![ROBOT_NAME].local run -it --memory="800m" --memory-swap="1.8g" --net host --privileged -v /data:/data --name kmeans duckietown/devel-kmeans-unsup:master18 /bin/bash

**Step 2**: Launch the camera_kmeans launch file

    duckiebot $ roslaunch pi_camera camera_kmean.launch veh:=![ROBOT_NAME] k:=![k] reduction:=![reduction]

`k` is number of centroids of the k-means algorithm with default of 3 <br/>
`reduction` is size reduction factor by downsampling and it's default is 4 <br/>

We suggest to open camera_kmeans.launch file and look at the flow of data in the node file.
camera_kmean.launch file is located under /catkin_ws/src/05-teleop/pi_camera/launch

**Step 3**:

    laptop $ dts start_gui_tools ![ROBOT_NAME]
    root@user $ rqt_image_view

This will start a rqt terminal, where you can see the image output after the k-means algorithm.

## Troubleshooting {#demo-k-means-troubleshooting}

Symptom: Cannot see image in the rqt.

Resolution: Check that you are receiving the compressed image using rostopic list on the duckiebot.

Symptom: Image is too slow

Resolution: Thats because k-means is computationally intensive. Increase the reduction parameter and check if you can run it faster
