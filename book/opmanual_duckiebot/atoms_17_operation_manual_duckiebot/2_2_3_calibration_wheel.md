# Calibration - Wheels {#wheel-calibration status=ready}

<div class='requirements' markdown='1'>

Requires: You can make your robot move as described in [](#sec:rc-control).

Results:  Calibrate the wheels of the Duckiebot such that it goes in a straight line
when you command it to. Set the maximum speed of the Duckiebot.

</div>

Want to learn more about odometry and odometry calibration? Check out our massive open online course resources: [video](videolink), [theory, activities and exercises](theorylink).

[videolink]: https://vimeo.com/manage/videos/580764763

[theorylink]: https://github.com/duckietown/mooc-exercises/tree/daffy/modcon

## Step 1: Make your robot move

Follow instructions in [](#sec:rc-control) to make your robot move, either with a joystick or the keyboard (preferred).

## Step 2: See how the robot really moves

There is a lot going on between pressing &#8593; on the keyboard and your Duckiebot moving. To get a better view of what is going on, we need a terminal where the action is happening:

    laptop $ dts start_gui_tools ![hostname]

Duckietown uses [ROS](https://www.ros.org/) to move data around. To determine if the command above worked, type:

    $ rostopic list

If everything is right, you will see a list of ROS _topics_ currently active on your Duckiebot. If you know ROS, here you can use ROS commands at will. If you are not familiar with ROS, note that each of these _topics_ might carry _messages_, i.e., actual data. You can, e.g., "listen" to the data inside each topic. For example:

    $ rostopic echo /!hostname/camera_node/image/compressed

will show you incoming images as the Duckiebot sees them!

## Step 3: Perform the calibration

### Calibrating the `trim` parameter

The trim parameter is set to $0.00$ by default, under the assumption that both motors and wheels are perfectly identical. You can change the value of the trim parameter by running the command:

    duckiebot-container $ rosparam set /![hostname]/kinematics_node/trim ![trim value]

Use some tape to create a straight line on the floor ([](#fig:wheel_calibration_line)).

<div figure-id="fig:wheel_calibration_line" figure-caption="Straight line useful for wheel calibration">
     <img src="wheel_calibration_line.jpg" style='width: 30em'/>
</div>

Place your Duckiebot on one end of the tape. Make sure that the Duckiebot is
perfectly centered with respect to the line.

Command your Duckiebot to go straight for about 2 meters. Observe the Duckiebot
from the point where it started moving and annotate on which side of the tape
the Duckiebot drifted ([](#fig:wheel_calibration_lr_drift)).

<div figure-id="fig:wheel_calibration_lr_drift" figure-caption="Left/Right drift">
  <img src="wheel_calibration_lr_drift.jpg" style='width: 30em'/>
</div>


Measure the distance between the center of the tape and the center of the axle of
the Duckiebot after it traveled for about 2 meters ([](#fig:wheel_calibration_measuring_drift)).

Make sure that the ruler is orthogonal to the tape.

<div figure-id="fig:wheel_calibration_measuring_drift" figure-caption="Measure the amount of drift after 2 meters run">
     <img src="wheel_calibration_measuring_drift.jpg" style='width: 30em'/>
</div>

If the Duckiebot drifted by less than $10$ centimeters you can stop calibrating the trim parameter. A drift of $10$ centimeters in a $2$ meters run is good enough for Duckietown. If the Duckiebot drifted by more than $10$ centimeters, continue with the next step.

If the Duckiebot drifted to the left side of the tape, decrease the value of $r$, by running, for example:

    duckiebot-container $ rosparam set /![hostname]/kinematics_node/trim -0.1

If the Duckiebot drifted to the right side of the tape, increase the value of
$r$, by running, for example:

    duckiebot-container $ rosparam set /![hostname]/kinematics_node/trim 0.1



Repeat this process until the robot drives straight.

Want to learn more about odometry and odometry calibration? Check out our massive open online course resources: [video](videolink), [theory, activities and exercises](theorylink).

### Calibrating the `gain` parameter

The gain parameter is set to $1.00$ by default. You can change its value by
running the command:

    duckiebot-container $ rosparam set /![hostname]/kinematics_node/gain ![gain value]

<!-- You won't really know if it's right until you verify it though! onto the next section

### Verify your calibration {#verify-kinematic-calibration status=beta}

Construct a calibration station similar to the one in [](#fig:kinematic_calibration):

<div figure-id="fig:kinematic_calibration" figure-caption="Kinematic calibration verification setup">
     <img src="kinematic_calibration1.jpg" style='width: 30em'/>
     <img src="kinematic_calibration3.pdf" style='width: 30em'/>
     <img src="kinematic_calibration2.jpg" style='width: 30em'/>
</div>

Note: In the sketch of the setup, the light green sections represent the interlocking parts of the tiles.

The following are the specs for this 3x1 mat "runway":

 - For red and white tape, use the one provided in your Duckietown kit

 - Blue/Black tape is ignored by Duckiebots, you can use anything of any color (expect red/white/yellow) to mark the positions.

 - Red line as close to the edge without crossing the interlocking bits.

 - Blue/Black line 8 cm from red line and parallel to it.

 - White lines on the edge without intersecting the interlocking bits

 - Yellow line in the middle of the white lines

 - Blue/black start position is ~3-4 cm from the edge (not including the interlocking bits)


Place your robot as shown in [](#fig:kinematic_calibration).

In the open shell execute:

    duckiebot-container $ roslaunch indefinite_navigation calibrate_kinematics.test veh:=![veh_name]

Then open a second shell with:

    laptop $ docker -H ![hostname].local exec -it demo_base /bin/bash

And inside of it run the test script with:

    duckiebot-container $ rosrun indefinite_navigation  test_kinematics.py

You should see your robot drive down the lane. If it is calibrated properly, you will see a message saying that it has `PASSED`, otherwise it is `FAILED` and you should adjust your gains based on what you observe and try again. You can use the shell in which you ran the `rosrun` command to modify the calibration. -->

### Store the calibration

When you are all done, save the parameters by running:

    duckiebot-container $ rosservice call /![hostname]/kinematics_node/save_calibration

The first time you save the parameters, this command will create the file


### Final check to make sure it's stored

The calibration result is saved on your Duckiebot:

```
/data/config/calibrations/kinematics/![hostname].yaml
```


You can view or download the calibration file using the Dashboard running at `http://![hostname].local` under `File Manager` in the sidebar on the left, navigating to `config/calibrations/kinematics/![hostname].yaml`.

<!--

Read more [here](#dashboard-tutorial-files)

-->

### Additional information

There are additional parameters you can to play around with to get a better driving experience. You can learn about odometry and odometry calibration here: [video](videolink), [theory, activities and exercises](theorylink).