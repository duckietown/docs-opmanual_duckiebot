# Hardware test {#setup-troubleshooting-hw-test status=ready}

If you have assembled your robot but you are not sure if you did it correctly, i.e. are the motors connected in the right way, is the camera attached properly, there is a small tool that you can use to test that.

The procedure consists of downloading and flashing a test image on an SD card. If you have only one SD card, you might want to do that before you do the steps in [](#setup-duckiebot).

Note: These instructions are valid for the DB18 version of the Duckiebot with a Raspberry Pi 3B+. If you have a different version of the hardware, you might not observe the expected behavior.

## Test procedure

1. Download this SD card [image](https://u.pcloud.link/publink/show?code=XZWVLNkZNoKQp6aU46hjTg1cmewUb7eiN5YV)

2. Extract the `.img` image from the archive.

3. Flash the image on an SD card. If you use Ubuntu, you can use the USB Image Writer tool that it comes with.

    Note: Make sure you write it to the right device! You can damage your system if you select a different device!
    
4. Now put the SD card in the assembled robot and power it up. 

5. Put the robot on the ground. 

If everything is successful within 30 to 60 seconds you should your robot's lights start to change. If your robot is assembled correctly you should observe the following behavior:

1. All LEDs are __white__ for 2 seconds.

2. All LEDs are __green__ for 2 seconds.

3. All LEDs are __red__ for 2 seconds.

4. The LEDs on the __right__ side of the robot will be __red__, the rest are white, and the robot is turning __right__ (i.e _into_ the direction of the red LEDs).

5. The LEDs on the __left__ side of the robot will be __red__, the rest are white, and the robot is turning __left__ (i.e _into_ the direction of the red LEDs).

6. The robot stops.

7. After a few seconds all LEDs are green.

8. End of the test. Everything appears to be assembled correct.

This is a video of what you should observe if your robot is assembled correctly:

<dtvideo src="vimeo:438103873"/>


## Troubleshooting

Symptom: The robot doesn't move in the right direction or moves backwards instead of forward.

Resolution: You have attached the motor wires in the wrong sockets. You will have to open the robot, fix that and test again.

Symptom: At the end of the procedure the LEDs don't turn green and the LED under the camera is red.

Resolution: This means that the test procedure could not obtain an image from the camera. Check that your camera cable is attached correctly at both ends and that your camera is not broken.
