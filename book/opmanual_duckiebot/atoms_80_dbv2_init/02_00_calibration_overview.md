# Calibration {#dbv2-calibration status=beta}

<div class='requirements' markdown='1'>

Requires: A Duckiebot set up according to [](#dbv2-setup)

Results: A calibrated DBV2.

</div>

`DBV2` follows the same calibration procedure as `DB18` for the camera, as described in
[](#camera-calib). The camera on `DBV2` is placed at the exact same hight and
orientation as `DB18`, so you can use the same calibration grid.

You must also calibrate the new sensors on `DBV2`.

## Calibration overview

Unlike camera calibration, the sensor calibration on DBV2 is not yet packaged in an easy-to-run demo. Therefore,
the procedure to run these calibrations is not intuitive. Every calibration will follow a similar process:

**Step 1**: Stop the `duckiebot-interface-dbv2` container: Each sensor calibration reads directly from the sensor. If
 `duckiebot-interface-dbv2` is also running, and also trying to read from the same sensor, there can be conflicts
 on the communication bus.

**Step 2**: Run a docker command: The docker commands given below create temporary containers for calibration. They use
 the following flags:
     - `--privileged`: Gives the container access to hardware and communication buses (I2C, GPIO, etc.)
     - `-it`: Starts the container interactively, because the calibration procedures need user input
     - `-v /data:/data`: Attaches the folder `/data` on the host raspberry pi to the location `/data` within the
       container. This is needed by all duckietown containers to store calibration files that persist between
       containers.
     - `--network=host`: This is used by all duckietown containers to allow them to communicate with the ROS master
     - `--rm`: Remove the container when done. Because calibration only needs to be done once for each sensor,
       it makes sense to clean up the containers when done.
 
**Step 3**: Give input on the command line: The calibration procedures require you to provide some data that you measure,
 to compare to the data the sensors measure and adjust accordingly.
