# Hardware assembly verification {#setup-troubleshooting-hw-test-db18 status=ready}

If you have assembled your Duckiebot `DB18` but you are not sure if you did it correctly, you can use this tool test that. This step is not mandatory but do serve as a good troubleshooting tool.

<div class='requirements' markdown="1">

Requires: Duckiebot `DB18` parts. Here is an [overview of all existing configurations](#duckiebot-configurations).

Requires: A micro SD card.

Requires: 20 minutes.

Result: Confirmation of correct `DB18` hardware assembly, or troubleshooting.

</div>

Note: The procedure consists of downloading and flashing a test image on an SD card. If you have only one SD card, you might want to do that before you do the steps in [](#setup-duckiebot).

## Test procedure

1. Download this SD card [image](https://duckietown-public-storage.s3.amazonaws.com/disk_image/DB18_hw_test_image_13_07_2020_shrunk.img.gz)

2. Extract the `.img` image from the archive.

3. Flash the image on an SD card. If you use Ubuntu, you can use the USB Image Writer tool that it comes with.

    Note: Make sure you write it to the right device! You can damage your system if you select a different device!

4. Now put the SD card in the assembled robot and power it up.

5. Put the robot on the ground.

If everything is successful within 30 to 60 seconds you should your robot's lights start to change. If your robot is assembled correctly you should observe the following behavior:

1. All LEDs are __green__ for 2 seconds.

2. All LEDs are __red__ for 2 seconds.

3. The LEDs on the __right__ side of the robot will be __red__, the rest are white, and the robot is turning __right__ (i.e _into_ the direction of the red LEDs).

4. The LEDs on the __left__ side of the robot will be __red__, the rest are white, and the robot is turning __left__ (i.e _into_ the direction of the red LEDs).

5. The robot stops.

6. After a few seconds all LEDs are green.

7. End of the test. Everything appears to be assembled correct.

This is a video of what you should observe if your robot is assembled correctly:

<dtvideo src="vimeo:438103873"/>  


## Troubleshooting

Symptom: The robot doesn't move in the right direction or moves backwards instead of forward.

Resolution: You have attached the motor wires in the wrong sockets. You will have to open the robot, fix that and test again.

Symptom: At the end of the procedure the LEDs don't turn green and the LED under the camera is red.

Resolution: This means that the test procedure could not obtain an image from the camera. Check that your camera cable is attached correctly at both ends and that your camera is not broken.
