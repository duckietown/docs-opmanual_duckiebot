# Wheel calibration {#wheel-calibration status=ready}

Assigned: Jacopo Tani

<div class='requirements' markdown='1'>

Requires: You can make your robot move as described in [](#sec:rc-control).

Results:  Calibrate the wheels of the Duckiebot such that it goes in a straight line
when you command it to. Set the maximum speed of the Duckiebot.

</div>

Comment: It might be helpful to talk about the ROS Parameter Server here, or at least
reference another page. -AD

For the theoretical treatment of the odometry calibration see [](+learning_materials#odometry_calibration).



## Step 1: Make your robot move

Follow instructions in [](#sec:rc-control) to make your robot movable either with a joystick or the keyboard. 

Note: Whatever container you used in [](#sec:rc-control) needs to have the `-v /data:/data` flag set or the calibration will not persist on your Duckiebot.


## Step 2


### Docker 

If you just finished the [camera calibration step](#camera-calib) then you have a docker terminal ready to use on your laptop.  


### Docker + ROS


Get a base container running on your robot if you don't have one already:


    duckiebot $ docker -H ![Duckiebot name].local run -it --net host --privileged  duckietown/rpi-duckiebot-base:master18



## Step 3: Perform the Calibration


### Calibrating the `trim` parameter

The trim parameter is set to $0.00$ by default, under the assumption that both motors and wheels are perfectly identical. You can change the value of the trim parameter by running the command:

    duckiebot $ rosservice call /![robot name]/inverse_kinematics_node/set_trim -- ![trim value]

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

    duckiebot $ rosservice call /![robot name]/inverse_kinematics_node/set_trim -- -0.1

If the Duckiebot drifted to the right side of the tape, increase the value of
$r$, by running, for example:

    duckiebot $ rosservice call /![robot name]/inverse_kinematics_node/set_trim -- 0.1



Repeat this process until the robot drives straight



### Calibrating the `gain` parameter

The gain parameter is set to $1.00$ by default. You can change its value by
running the command:

    duckiebot $ rosservice call /![robot name]/inverse_kinematics_node/set_gain -- ![gain value]

You won't really know if it's right until you verify it though! onto the next section

### Verify your calibration {#verify-kinematic-calibration}

Construct a calibration station similar to the one in [](#fig:kinematic_calibration):

<div figure-id="fig:kinematic_calibration" figure-caption="Kinematic calibration verification setup">
     <img src="kinematic_calibration1.jpg" style='width: 30em'/>
     <img src="kinematic_calibration2.jpg" style='width: 30em'/>
</div>

The following are the specs for this 3x1 mat "runway":

 - Red line as close to the edge without crossing the interlocking bits

 - Blue/Black line 8 cm from red line and parallel to it.

 - White lines on the edge without intersecting the interlocking bits

 - Yellow line in the middle of the white lines

 - Blue/black start position is ~3-4 cm from the edge (not including the interlocking bits)


Place your robot as shown in [](#fig:kinematic_calibration).

On your robot execute:

    duckiebot $ cd ![duckietown root]
    duckiebot $ make hw-test-kinematics

You should see your robot drive down the lane. If it is calibrated properly, you will see a message saying that it has `PASSED`, otherwise it is `FAILED` and you should adjust your gains based on what you observe and try again.

### Store the calibration

When you are all done, save the parameters by running:

    duckiebot $ rosservice call /![robot name]/inverse_kinematics_node/save_calibration

The first time you save the parameters, this command will create the file


### Final Check to make sure it's stored


Assuming your are running an HTTP server, point your browser to `http://![Duckiebot name].local:8082/config/calibrations/kinematics/![Duckiebot name].yaml`
