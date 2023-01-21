# Calibration - Kinematics {#dbv2-calib-kinematics status=beta}

<div class='requirements' markdown='1'>

Requires: A Duckiebot set up according to [](#dbv2-setup)

Results: Kinematics are calibrated.

</div>

The kinematics node does not have an automatic calibration procedure like the sensors have. Instead, you should
open up a graph of the demanded velocity vs. the actual velocity, and tweak the parameters until you get good
performance.

**Step 1**: Follow the steps in [](#rc-control) to open a virtual joystick.

**Step 2**: Open GUI tools:

    $ dts start_gui_tools ![duckiebot name] --base_image duckietown/dt-core:daffy

**Step 3**: Within GUI tools, run the command `rqt_plot &amp;`, and add the following topics:

 - `/![duckiebot_name]/car_cmd_switch_node/cmd/v`: Shows the demanded velocity of the Duckiebot
 - `/![duckiebot name]/encoder_node/encoder_velocity/vel_encoder`: Shows the current velocity measured by
   the wheel encoder.
 - `/![duckiebot name]/wheels_driver_node/wheels_cmd_dbv2/vel_wheel`: The duty cycle sent to the DC motor. 
   This is constrained to the range `[-1, +1]`.

**Step 4**: Drive the Duckiebot around using the joystick, and observe the graph. If the `vel_wheel` is consistently
at or near 1, then you should turn down the joystick speed gain:

    $ rosparam set /dbv2optimus/joy_mapper_node/speed_gain 0.3

If the wheel velocity is saturated at 1, then it will be impossible to tune the speed controller.

**Step 5**: Reset all of the necessary parameters, by running these commands within GUI tools:

    $ rosparam set /![duckiebot name]/kinematics_node/k_P 0
    $ rosparam set /![duckiebot name]/kinematics_node/k_I 0

**Step 6**: Adjust the parameter `kinematics_node/k_wheel` until the encoder velocity roughly matches
the set speed. In [](#dbv2-kinem-k-wheel), there are three separate sections. From left to right:

 - Duckiebot is driving too fast: `k_wheel` is too low.
 - Duckiebot is driving too slow: `k_wheel` is too high.
 - Duckiebot is driving at the correct speed: `k_wheel` is correct.
 

<figure id="dbv2-kinem-k-wheel">
    <figcaption>Tuning k_wheel</figcaption>
    <img style='width:100%' src="dbv2_init_images/adjust_k_wheel.png" alt="Tuning k_wheel"/>
</figure>

**Step 7**: Adjust the parameter `kinematics_node/k_P` to be as high as possible, without introducing oscillations.
[](#dbv2-kinem-k-p) shows two separate sections, from left to right:

 - Speed is oscillating: `k_P` is too high
 - Speed is not oscillation: `k_P` is correct
 
<figure id="dbv2-kinem-k-p">
    <figcaption>Tuning k_P</figcaption>
    <img style='width:100%' src="dbv2_init_images/adjust_k_p.png" alt="Tuning k_p"/>
</figure>

**Step 7**: Adjust the parameter `kinematics_node/k_I` to be as high as possible, without introducing oscillations.
[](#dbv2-kinem-k-i) shows two separate sections, from left to right:

 - Speed is oscillating: `k_I` is too high
 - Speed is not oscillating: `k_I` is correct
 
<figure id="dbv2-kinem-k-i">
    <figcaption>Tuning k_I</figcaption>
    <img style='width:100%' src="dbv2_init_images/adjust_k_i.png" alt="Tuning k_i"/>
</figure>

**Step 8**: Save the calibration by running:

    $ rosservice call /![duckiebot name]/kinematics_node/save_calibration

## Validation

To validate this tuning, keep the same graph open. Drive the duckiebot around in the city, and pay attention
to the measured speed when going around turns. The duckiebot tends to slow down slightly around turns, and so
the kinematics node should compensate by slightly increasing the output to the motors.

## Demo

By default, all demos on DBV2 use this speed control. See [](#dbv2-demos)
