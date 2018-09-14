# Wheel calibration {#wheel-calibration status=ready}

Assigned: Andrea Daniele

<div class='requirements' markdown='1'>

Requires: You can run the joystick demo remotely. The procedure is documented
in [](#rc-launched-remotely).

Results:  Calibrate the wheels of the Duckiebot such that it goes in a straight line
when you command it to. Set the maximum speed of the Duckiebot.

</div>

TODO: Jacopo Tani - separate theory from demo instructions

## Introduction

The motors used on the Duckiebots are called "Voltage-controlled motors".

This means that the velocity of each motor is directly proportional to the voltage it is subject to. Even though we use the same model of motor for left and right wheel, they are not exactly the same. In particular, every motor responds to a given voltage signal in a slightly different way. Similarly, the wheels that we are using look "identical", but they might be slightly different.

If you drive the Duckiebot around using the joystick, you might notice that it doesn’t really go in a straight line when you command it to. This is due to those small differences between the motors and the wheels explained above.

Different motors can cause the left wheel and right wheel to travel at different speed even though the motors received the same command signal. Similarly, different wheels travel different distances even though the motors made the same rotation.

Comment: It might be helpful to talk about the ROS Parameter Server here, or at least
reference another page. -AD


## What is the Calibration step?

We can counter this behavior by *calibrating* the wheels. A calibrated Duckiebot
sends two different signals to left and right motor such that the robot moves in
a straight line when you command it to.

The relationship between linear and angular velocity of the robot and the velocities
of left and right motors are:

$$
\begin{align*}
    v_{\text{right}} &= (g + r) \cdot (v + \dfrac{1}{2} \omega l ) \\
    v_{\text{left}} &= (g - r) \cdot (v - \dfrac{1}{2} \omega l )
\end{align*}
$$

where $v_{\text{right}}$ and $V_{\text{left}}$ are the velocities of the two motors, $g$ is called *gain*, $r$ is called *trim*, $v$ and $\omega$ are the desired linear and the angular velocity of the robot, and $l$ is the distance between the two
wheels. The gain parameter $g$ controls the maximum speed of the robot.

With $g > 1.0$, the vehicle goes faster given the same velocity command, and for $g < 1.0$ it goes slower. The trim parameter $r$ controls the balance between the two motors.

With $r > 0$, the right wheel will turn slightly more than the left wheel given the same velocity command; with $r < 0$, the left wheel will turn slightly more the right wheel.

TODO for Jacopo Tani: It might be helpful to add the differential equations that relate velocities and voltages of the motors. -AD


## Perform the Calibration

### Calibrating the `trim` parameter

The trim parameter is set to $0.00$ by default, under the assumption that both motors and wheels are perfectly identical. You can change the value of the trim parameter by running the command:

    duckiebot $ rosservice call /![robot name]/inverse_kinematics_node/set_trim -- ![trim value]

To calibrate the trim parameter use the following steps:

#### Step 1

Make sure that your Duckiebot is ON and connected to the network.

#### Step 2

On your Duckiebot, launch the joystick process:

    duckiebot $ roslaunch duckietown joystick.launch veh:=![robot name]

#### Step 3

Use some tape to create a straight line on the floor ([](#fig:wheel_calibration_line)).

<div figure-id="fig:wheel_calibration_line" figure-caption="Straight line useful for wheel calibration">
     <img src="wheel_calibration_line.jpg" style='width: 30em'/>
</div>


#### Step 4

Place your Duckiebot on one end of the tape. Make sure that the Duckiebot is
perfectly centered with respect to the line.

#### Step 5

Command your Duckiebot to go straight for about 2 meters. Observe the Duckiebot
from the point where it started moving and annotate on which side of the tape
the Duckiebot drifted ([](#fig:wheel_calibration_lr_drift)).

<div figure-id="fig:wheel_calibration_lr_drift" figure-caption="Left/Right drift">
  <img src="wheel_calibration_lr_drift.jpg" style='width: 30em'/>
</div>

#### Step 6

Measure the distance between the center of the tape and the center of the axle of
the Duckiebot after it traveled for about 2 meters ([](#fig:wheel_calibration_measuring_drift)).

Make sure that the ruler is orthogonal to the tape.

<div figure-id="fig:wheel_calibration_measuring_drift" figure-caption="Measure the amount of drift after 2 meters run">
     <img src="wheel_calibration_measuring_drift.jpg" style='width: 30em'/>
</div>

If the Duckiebot drifted by less than $10$ centimeters you can stop calibrating the trim parameter. A drift of $10$ centimeters in a $2$ meters run is good enough for Duckietown. If the Duckiebot drifted by more than $10$ centimeters, continue with the next step.


#### Step 7

If the Duckiebot drifted to the left side of the tape, decrease the value of $r$, by running, for example:

    duckiebot $ rosservice call /![robot name]/inverse_kinematics_node/set_trim -- -0.1

In order to run this command you should create another $ssh$ connection to the duckiebot as the joystick process should be still running.

#### Step 8

If the Duckiebot drifted to the right side of the tape, increase the value of
$r$, by running, for example:

    duckiebot $ rosservice call /![robot name]/inverse_kinematics_node/set_trim -- 0.1

In order to run this command you should create another $ssh$ connection to the duckiebot as the joystick process should be still running.

#### Step 9

Repeat the steps 4-8.

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

    ![DUCKIEFLEET_ROOT]/calibrations/kinematics/![robot name].yaml

#### Committing {#committing-calib status=outdated}

You can add and commit it to the repository. Then you should create a pull request in the [duckiefleet repo](https://github.com/duckietown/duckiefleet).
