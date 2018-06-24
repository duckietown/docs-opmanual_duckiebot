# Camera calibration {#camera-calib-old status=draft}

Note: This camera calibration procedure is outdated. See [](#camera-calib) for the updated procedure.

<div class='requirements' markdown='1'>

Requires: You can see the camera image on the laptop. The procedure is documented in
[](#rc-cam-launched-remotely).

Requires: You have all the repositories cloned properly and you have your environment variables set properly.

Results: Calibration for the robot camera.

</div>

## Intrinsic calibration

### Setup

Download and print a PDF of the calibration checkerboard
([A4 intrinsic](github:org=duckietown,repo=duckiefleet,path=calibrations/Calibration_pattern_A4_intrinsic.pdf), [A3 extrinsic](github:org=duckietown,repo=duckiefleet,path=calibrations/calibration_pattern_A3.pdf), [US Letter](github:org=duckietown,repo=Software,path=duckietown/config/baseline/calibration/camera_intrinsic/calibration_pattern.pdf)).
Fix the checkerboard to a planar surface.

<div figure-id="fig:calibration_checkerboard" figure-caption="">
     <img src="calibration_checkerboard.png" style='width: 20em'/>
</div>

Note: the squares must have side equal to 0.031 m = 3.1 cm.


### Calibration

Make sure your Duckiebot is on, and both your laptop and Duckiebot are connected to the duckietown network.

#### Step 1


Open two terminals on the laptop.

#### Step 2

In the first terminal, log in into your robot using SSH and launch the camera process:

    duckiebot $ cd ![duckietown root]
    duckiebot $ source environment.sh
    duckiebot $ roslaunch duckietown camera.launch veh:=![robot name] raw:=true

#### Step 3

In the second laptop terminal run the camera calibration:

    laptop $ cd ![duckietown root]
    laptop $ source environment.sh
    laptop $ source set_ros_master.sh ![robot name]
    laptop $ roslaunch duckietown intrinsic_calibration.launch veh:=![robot name]

You should see a display screen open on the laptop ([](#fig:intrinsic_callibration_pre)).

<div figure-id="fig:intrinsic_callibration_pre" figure-caption="">
     <img src="intrinsic_callibration_pre.png" style='width: 30em'/>
</div>

Position the checkerboard in front of the camera until you see colored lines
overlaying the checkerboard. You will only see the colored lines if the entire
checkerboard is within the field of view of the camera.

You should also see
colored bars in the sidebar of the display window. These bars indicate the
current range of the checkerboard in the camera's field of view:

- X bar: the observed horizontal range (left - right)
- Y bar: the observed vertical range (top - bottom)
- Size bar: the observed range in the checkerboard size (forward - backward from the camera direction)
- Skew bar: the relative tilt between the checkerboard and the camera direction

Also, make sure to focus the image by rotating the mechanical focus ring on the lens of the camera.

Comment: Do not change the focus during or after the calibration, otherwise your calibration is no longer valid. I'd also suggest to not to use the lens cover anymore; removing the lens cover changes the focus. -MK

Now move the checkerboard right/left, up/down, and tilt the checkerboard
through various angles of relative to the image plane. After each movement,
make sure to pause long enough for the checkerboard to become highlighted. Once
you have collected enough data, all four indicator bars will turn green. Press
the "CALIBRATE" button in the sidebar.

Calibration may take a few moments. Note that the screen may dim. Don't worry, the calibration is working.

<div figure-id="fig:intrinsic_calibration_calibratestep" figure-caption="">
 <img src="intrinsic_calibration_calibratestep.png" style='width: 30em'/>
</div>

### Save the calibration results

If you are satisfied with the calibration, you can save the results by pressing the "COMMIT" button in the side bar. (You never need to click the "SAVE" button.)

<div figure-id="fig:intrinsic_calibration_commit" figure-caption="">
     <img src="intrinsic_calibration_commit.png" style='width: 30em'/>
</div>

This will automatically save the calibration results on your Duckiebot:

    ![duckiefleet root]/calibrations/camera_intrinsic/![robot name].yaml

#### Step 4

Now let's push the `![robot name].yaml` file to the git repository. You can stop the `camera.launch` terminal with <kbd>Ctrl</kbd>-<kbd>C</kbd> or open a new terminal in Byobu with <kbd>F2</kbd>.

Update your local git repository:

    duckiebot $ cd ![duckiefleet root]
    duckiebot $ git pull
    duckiebot $ git status

You should see that your new calibration file is uncommitted. You need to commit the file to your branch.

    duckiebot $ git checkout -b ![Github username]-devel
    duckiebot $ git add calibrations/camera_intrinsic/![robot name].yaml
    duckiebot $ git commit -m "add ![robot name] intrinsic calibration file"
    duckiebot $ git push origin ![Github username]-devel

Before moving on to the extrinsic calibration, make sure to kill all running processes by pressing <kbd>Ctrl</kbd>-<kbd>C</kbd> in each of the terminal windows.

## Extrinsic calibration {#camera-calib-extrinsics}

### Setup {#camera-calib-extrinsics-setup}

Arrange the Duckiebot and checkerboard according to [](#fig:extrinsic_setup). Note that the axis of the wheels should be aligned with the y-axis ([](#fig:extrinsic_setup)).

<div figure-id="fig:extrinsic_setup" figure-caption="">
  <img src="extrinsic_setup.jpg" style='width: 30em'/>
</div>


[](#fig:extrinsic_view) shows a view of the calibration checkerboard from the Duckiebot. To ensure proper calibration there should be no clutter in the background and two A4 papers should be aligned next to each other.

<div figure-id="fig:extrinsic_view" figure-caption="">
  <img src="extrinsic_view.jpg" style='width: 30em'/>
</div>

### Calibration procedure {#camera-calib-extrinsics-procedure}


#### Step 1

Log in into your robot using SSH and launch the camera:

    duckiebot $ cd ![duckietown root]
    duckiebot $ source environment.sh
    duckiebot $ roslaunch duckietown camera.launch veh:=![robot name] raw:=true

#### Step 2

Run the `ground_projection_node.py` node on your laptop:

    laptop $ cd ![duckietown root]
    laptop $ source environment.sh
    laptop $ source set_ros_master.sh ![robot name]
    laptop $ roslaunch ground_projection ground_projection.launch  veh:=![robot name] local:=true

#### Step 3

Check that everything is working properly. In a new terminal (with environment sourced and ros_master set to point to your robot as above)

    laptop $ rostopic list

You should see new ros topics:

    /![robot name]/camera_node/camera_info
    /![robot name]/camera_node/framerate_high_switch
    /![robot name]/camera_node/image/compressed
    /![robot name]/camera_node/image/raw
    /![robot name]/camera_node/raw_camera_info

The ground_projection node has two services. They are not used during operation. They just provide a command line interface to trigger the extrinsic calibration (and for debugging).

    laptop $ rosservice list

You should see something like this:

    ...
    /![robot name]/ground_projection/estimate_homography
    /![robot name]/ground_projection/get_ground_coordinate
    ...

If you want to check whether your camera output is similar to the one at the [](#fig:extrinsic_view) you can start `rqt_image_view`:

    laptop $ rosrun rqt_image_view rqt_image_view

In the `rqt_image_view` interface, click on the drop-down list and choose the image topic:

    /![robot name]/camera_node/image/compressed

#### Step 4

Now you can estimate the homography by executing the following command (in a new terminal):

    laptop $ rosservice call /![robot name]/ground_projection/estimate_homography

This will do the extrinsic calibration and automatically save the file to your laptop:

    ![duckiefleet root]/calibrations/camera_extrinsic/![robot name].yaml

As before, add this file to your local Git repository on your laptop, push the changes to your branch and do a pull request to master.
Finally, you will want to update the local repository on your Duckiebot.
