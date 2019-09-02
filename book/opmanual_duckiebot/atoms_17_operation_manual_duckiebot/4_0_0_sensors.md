# Sensors

This page explains the sensors of the `DBv2`.

## Time of Flight Sensor (ToF)
To measure distances, a time of flight sensor can be used. The sensor sends out a signal and measures the time a signal needs to return. Knowing the propagation speed of the signal and the time it needs to come back, it is possible to calculate the travel distance. For the `DBv2` the spark fun RFD77402 sensor is used. That sensor has a distance range of 2 m and 55° in angle. According to the datasheet the precision is (+/-) 10%. Testing the precision by our self, we could find an error of less than 10% for distances between 5-100 cm. By a distance under 5 cm we got an error of 10-13% and measuring a distance over one meter gives errors which are much higher than 10% (the distance of 150 cm to a shiny box had an error of 30%).

<figure>
    <figcaption>The keyboard control graphical user interface</figcaption>
    <img style='width:8em' src="RFD77402.jpeg"/>
</figure>

To get the measurement of the time of flight call the rosservice tof_node with the position name of the sensor you want to measure the distance from

    $docker -H [DUCKIEBOT_NAME].local run --privileged -it --net host --rm --name sensors-test duckietown/duckiebot-v2-interface

Open a second terminal:
    $docker -H [DUCKIEBOT_NAME].local exec -it sensors-test /bin/bash

    $. catkin_ws/devel/setup.bash
    $rosservice call /tof_measurement "sensor_position: '[position name]'";

This service returns the measured distance in mm, the confidence value and the valid pixels, which are values of the confidence register and depend on the detected signal amplitude. The sensor measures different distances, as the light beam gets reflected differently. If there is no object in a range of 2 m sometimes a distance was measured with less than 100 valid pixels. This happens because the light beam can also be reflected by the ground. (We advise you to check, if the valid pixel is a number greater then 100 when using the distance.) The last information the time of flight service returns is the time stamp when the measurement is taken.

Position names:
<col2 figure-id="tab:tof_positions" figure-caption="ToF position names" class="labels-row1">
    <span>Position name</span>
    <span>Explanation</span>
    <span>tof_fl</span>
    <span>ToF front left</span>
    <span>tof_fm</span>
    <span>ToF front middle</span>
    <span>tof_fr</span>
    <span>ToF front right</span>
    <span>tof_sl</span>
    <span>ToF side left</span>
    <span>tof_sr</span>
    <span>ToF side right</span>
    <span>tof_bl</span>
    <span>ToF side right</span>
    <span>tof_bl</span>
    <span>ToF back left</span>
    <span>tof_bm</span>
    <span>ToF back middle</span>
    <span>tof_br</span>
    <span>ToF back right</span>
</col2>

## Line Following Sensor (LF)

## Inertial Measurement Unit (IMU)

## Camera

## Wheel Encoder

## Sensor Suite
