# Making your Duckiebot move {#rc-control status=ready}

This page is for Duckiebots in `DB18` configuration. For previous year's instructions see [here](https://docs.duckietown.org/DT17/).

<div class='requirements' markdown='1'>

Requires: A Duckiebot in `DB18` configuration.

Requires: Laptop configured, according to [](#laptop-setup).

Requires: You have configured the Duckiebot as documented in [](#setup-duckiebot).

Results: You can make your robot move.

</div>


<!--Requires: You have created a Github account and configured public keys,
both for the laptop and for the Duckiebot. The procedure is documented in [](+software_reference#github-access).-->


## Option 1 - With the Duckietown Shell {#make-it-move_shell status=ready}




Assuming that your Duckiebot is [properly initialized](#setup-duckiebot), if you have a gamepad then plug the usb dongle into the raspberry pi of your duckiebot and run:

    $ dts duckiebot demo --demo_name joystick --duckiebot_name ![DUCKIEBOT_NAME] --image duckietown/dt-core:daffy

and you should be able to move it with the joystick.

If you would like to move your robot using your laptop, first also run the line above - it will start the joystick node on the duckiebot that will listen to commands from the joystick emulator. To start the emulator, you can then run

    $ dts duckiebot keyboard_control ![DUCKIEBOT_NAME] --base_image duckietown/dt-core:daffy

which, after startup should open the interface window that looks like:

<figure>
    <figcaption>The keyboard control graphical user interface</figcaption>
    <img style='width:8em' src="keyboard_gui.png"/>
</figure>

The following keys control the Duckiebot:

<col2 figure-id="tab:virtual_keyboard" figure-caption="Keyboard joystick functions" class="labels-row1">
    <span>Keys</span>
    <span>Function</span>
    <span>ARROW_KEYS</span>
    <span>Steer your Duckiebot</span>
    <span>q</span>
    <span>Quit</span>
    <span>a</span>
    <span>Turn on Lane Following</span>
    <span>s</span>
    <span>Stop Lane Following</span>
    <span>i</span>
    <span>Toggle Anti-instagram</span>
</col2>

Warning: This does not currently work on Mac OSX - need to fix


## The no-window way  (For Mac Users) 

There is some weird reason that messages published on Mac inside the container don't make it all the way to the robot. 

For those that don't want or can't do the above where a window pops up, do the following (which will run directly on the robot): 

    laptop $ dts duckiebot keyboard_control ![hostname] --cli --base_image duckietown/dt-core:daffy


### Troubleshooting

Symptom: The robot doesn't move

Resolution: Check that the `duckiebot-interface` is running

Open [the Portainer interface](#docker-setup-portainer-interface) and check the running containers. You should see one called `dt18_03_roscore_duckiebot-interface_1`.

You call also determine this by running:

    $ docker -H ![DUCKIEBOT_NAME].local ps

and look at the output to find the Duckiebot interface container and verify that it is running.

Resolution: One of the base images is out of date

Pull the base images on the Duckiebot:

    $ docker -H ![DUCKIEBOT_NAME].local pull duckietown/dt-core:daffy

and on the laptop:

    $ docker pull duckietown/dt-core:daffy


<!--
## Option 2 - with Docker  {#make-it-move_docker status=ready}


Verify that the container is running by either using [the Portainer interface](#docker-setup-portainer-interface)
or by using `docker ps`.


### Run the joystick demo

Use the following command to run the joystick demo:


    laptop $ docker -H ![hostname].local run -dit --privileged --name joystick --network=host -v /data:/data duckietown/rpi-duckiebot-joystick-demo:master18


### Controlling your robot with a joystick

If you have a joystick, you can use it to make your robot move.

Otherwise, you can use the following instructions to run the demo with
keyboard control.
-->

## Option 2: Using the dashboard {#setup-ros-websocket-image status=beta}

If you followed the instructions in [](#duckiebot-dashboard-setup), you
should have access to the Duckiebot dashboard.

You can open the browser and visit the page `http://![hostname].local/mission-control`.

This is the Mission Control page.
It is the page that lets you monitor and control your Duckiebot.
The top of the page should be similar to the following image,


<div figure-id="fig:dashboard_mission_control_auto" figure-caption="">
  <img src="dashboard_mission_control_auto.png" style='width: 35em'/>
</div>


The first thing to check to make sure that everything we have done so far
is correct, is the status of the **Bridge**, in the top-right corner of the page.
The label should show the status "**Bridge: Connected**" (as shown in the image above).
If the indicator reads "**Bridge: Closed**", it means that something went wrong
while launching the ROS websocket node above. In that case, start again from
the beginning of this section.

Note: Don't worry if one of the blocks is called "Camera" but you
don't see an image. We will get to that later.

This page will show you lateral and angular speed of your robot, and
a plot of left and right motor speed. Toggle the **Take over** switch
in the top-right corner of the page to gain control of your robot.
You will see that the background of the page will highlight and the
central plot will start moving.

You can now use the arrows on your keyboard to drive your Duckiebot.

**Did you know?**
The page contains 4 blocks by default.
Feel free to drag them around and rearrange them as you please.
You can also use the menu button of each block to resize them.

