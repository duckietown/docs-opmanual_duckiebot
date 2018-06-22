
# Camera calibration and validation {#camera-calib status=ready}

Here is an updated, more practical extrinsic calibration and validation procedure.


## Check out the experimental branch

Check out the branch `andrea-better-camera-calib`.


## Place the robot on the pattern {#camera-calib-jan18-extrinsics-setup}

Arrange the Duckiebot and checkerboard according to [](#fig:extrinsic_setup2). Note that the axis of the wheels should be aligned with the y-axis ([](#fig:extrinsic_setup2)).

<div figure-id="fig:extrinsic_setup2" figure-caption="">
  <img src="extrinsic_setup.jpg" style='width: 30em'/>
</div>


[](#fig:extrinsic_view2) shows a view of the calibration checkerboard from the Duckiebot. To ensure proper calibration there should be no clutter in the background and two A4 papers should be aligned next to each other.

<div figure-id="fig:extrinsic_view2" figure-caption="">
  <img src="extrinsic_view.jpg" style='width: 30em'/>
</div>

## Extrinsic calibration procedure {#camera-calib-jan18-extrinsics}

Run the following on the Duckiebot:

    duckiebot $ rosrun complete_image_pipeline calibrate_extrinsics

That's it!

No laptop is required.

You can also look at the output files produced, to make sure it looks reasonable.
It should look like [](#fig:calibrate_extrinsics1).


<div figure-id="fig:calibrate_extrinsics1" figure-caption="">
  <img src="2_2_2_camera/calibrate_extrinsics1.jpg" style='width: 90%'/>
</div>

Note the difference between the two types of rectification:

1. In `bgr_rectified` the rectified frame coordinates are chosen so that
the frame is filled entirely. Note the image is stretched - the April tags
are not square. This is the rectification used in the lane localization pipeline. It doesn't matter that the image is stretched, because the homography learned will account for that deformation.

2. In `rectified_full_ratio_auto` the image is not stretched. The camera matrix is preserved. This means that the aspect ratio is the same. In particular note the April tags are square. If you do something with April tags, you need this rectification.


## Camera validation by simulation {#camera-calib-jan18-simulation}

You can run the following command to make sure that the camera calibration is reasonable:

    duckiebot $ rosrun complete_image_pipeline validate_calibration

What this does is simulating what the robot should see, if the models were correct ([](#fig:validate_calibration_out1)).

<div figure-id="fig:validate_calibration_out1">
    <img style='width:40%' src="2_2_2_camera/validate_calibration_out1.jpg"/>
    <figcaption>Result of <code>validate_calibration</code>.</figcaption>
</div>

Then it also tries to localize on the simulated data ([](#fig:try_simulated_localization)).
It usual achieves impressive calibration results!

> Simulations are doomed to succeed.

<div figure-id="fig:try_simulated_localization">
    <img style='width:90%' src="2_2_2_camera/try_simulated_localization.jpg"/>
    <figcaption>Output of <code>validate_calibration</code>: localization
    in simulated environment.</figcaption>
</div>


## Camera validation by running one-shot localization {#camera-calib-jan18-oneshot}

Place the robot in a lane.

Run the following command:

    duckiebot $ rosrun complete_image_pipeline single_image_pipeline

What this does is taking one snapshot and performing localization on that single image.
The output will be useful to check that everything is ok.


### Example of correct results

[](#fig:oneshot1_all) is an example in which the calibration was correct, and the robot
localizes perfectly.

<div figure-id="fig:oneshot1_all">
    <img style='width:90%' src="2_2_2_camera/oneshot/oneshot1_all.jpg"/>
    <figcaption>Output when camera is properly calibrated.</figcaption>
</div>

### Example of failure

This is an example in which the calibration is incorrect.

Look at the output in the bottom left: clearly the perspective is distorted,
and there is no way for the robot to localize given the perspective points.

<div figure-id="fig:incorrect1">
    <img style='width:90%' src="2_2_2_camera/incorrect1.jpg"/>
    <figcaption>Output when camera not properly calibrated.</figcaption>
</div>


## The importance of validation

Validation is useful because otherwise it is hard to detect wrong calibrations.

For example, in 2017, a bug in the calibration made about 5 percent
of the calibrations useless ([](#fig:calibration_95_percent_success)), and people didn't notice for weeks (!).


<div figure-id="fig:calibration_95_percent_success">
    <img style='width:90%' src="2_2_2_camera/calibration_95_percent_success.jpg"/>
    <figcaption>In 2017, a bug in the calibration made about 5 percent
    of the calibrations useless.</figcaption>
</div>
