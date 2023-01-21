# Calibration - Line Follower Sensors {#dbv2-calib-lf status=beta}

<div class='requirements' markdown='1'>

Requires: A Duckiebot set up according to [](#dbv2-setup)

Requires: Line follower sensors

Results: Line follower sensors are calibrated.

</div>

The line follower sensors are the 4 sensors on the bottom of the front bumper of DBV2. They detect the brightness
of the surface over which they are positioned. This can be used to differentiate between the black road surface,
the white outer lines, the yellow center lines, and red intersection lines.

The sensors are very sensitive to their exact height above the ground, and this calibration procedure accounts
for that. After calibration is done, all 4 sensors will give the same brightness values for the same color.

**Step 1**: Use Portainer to stop the `duckiebot-interface-dbv2` container.

**Step 2**: Use the following command to launch the calibration procedure:

    $ docker -H ![duckiebot name].local run --privileged -it -v /data:/data \
      --name duckiebot-interface-calibration --network=host --rm \
      duckietown/dt-duckiebot-interface-dbv2:daffy-arm32v7 \
      roslaunch sensor_suite line_following_calibration.launch

**Step 3**: Follow the instructions on the command line. You will be asked to place the Duckiebot such that all 4 
line follower sensors are on a particular color.

The results of the calibration will be stored in 
`/data/config/calibrations/sensor_suite/line_follower/![duckiebot name].yaml`.

## Validation

To validate that the calibration worked, do the following:

**Step 1**: Start the `duckiebot-interface-dbv2` container in Portainer.

**Step 2**: Start GUI tools on your computer using Duckietown Shell:

    $ dts start_gui_tools ![duckiebot name] --base_image duckietown/dt-core:daffy

**Step 3**: Within GUI tools, run the command `rqt_plot`. A plot window should open.

**Step 4**: In the Plot window, select the topic `/![duckiebot name]/line_following_node/line_follower` and
click on the 'plus' button.

**Step 5**: Place the Duckiebot such that the sensors are over various colors on a city tile, and observe
the values in the plot. The values measured should be within +/- 0.2 of the ideal values given below.

<col2 class="labels-col1" figure-id="tab:idea-color-measures" 
        figure-caption="Ideal measurements for line follower sensors">
    <span>Yellow</span>
    <span>0.2</span>
    <span>White</span>
    <span>0.6</span>
    <span>Black</span>
    <span>1.0</span>
</col2>

## Demo

See [](#dbv2-demos-line-follower)
