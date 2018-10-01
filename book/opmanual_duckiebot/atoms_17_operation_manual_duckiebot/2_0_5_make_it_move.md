# Making your Duckiebot move {#rc-control status=ready}

Assigned: Breandan Considine, Liam Paull

This page is for the `DB18` configuration used in classes in 2018. For last year's instructions see [here](https://docs.duckietown.org/DT17/).

<div class='requirements' markdown='1'>

Requires: Laptop configured, according to [](#laptop-setup).

Requires: You have configured the Duckiebot as documented in [](#setup-duckiebot).

Requires: You have configured Docker communication as documented in [](#docker-setup).

Results: You can make your robot move.

</div>


<!--Requires: You have created a Github account and configured public keys,
both for the laptop and for the Duckiebot. The procedure is documented in [](+software_reference#github-access).-->



## Option 1 - Pure Docker {#make-it-move_docker status=beta}
 

### Run the `roscore` container 
    
Use the following command to run the container that contains `roscore`:

    laptop $ docker -H ![Duckiebot name].local run -dit --privileged --name roscore --net host --restart unless-stopped duckietown/rpi-ros-kinetic-roscore:master18
    
If this is the first time you run this, it might take some time to download the container.

Verify that the container is running by either using [the Portainer interface](#docker-setup-portainer-interface)
or by using `docker ps`. 
 


    
### Run the joystick demo

Use the following command to run the joystick demo:

    laptop $ docker -H ![Duckiebot name].local run -dit --privileged --name joystick --network=host -v /data:/data duckietown/rpi-duckiebot-joystick-demo:master18


### Controlling your robot with a joystick


If you have a joystick, you can use it to make your robot move.

Otherwise, you can use the following instructions to run the demo with 
keyboard control 


### Controlling your robot with your keyboard


##### OSX

On your laptop run:

    laptop $ dts keyboard_control ![Duckiebot name]

Note: Currently not working for Mac OSX - the window pops up but the robot won't move - this is expected for now

TODO: Liam fix this

This will pop open a window where you can use arrows to control the robot.

The following keys are supported:

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



## Option 2 - Docker + ROS {#make-it-move_docker_ros status=beta}


Run the base image on the duckiebot:

    duckiebot $ docker -H ![Duckiebot name].local run -it --net host --privileged --name base duckietown/rpi-duckiebot-base:master18 /bin/bash


Then when the container has started 

    container $  roslaunch duckietown joystick.launch veh:=![Duckiebot name]


#### Controlling your robot with a joystick

You can now use your joystick to move things around


#### Controlling your robot with the keyboard

If you have cloned the repo on your laptop and installed ROS, then you can start the keyboard controller interface with


    laptop $ make virjoy-![Duckiebot name]


## Option 3 - Pure ROS {#make-it-move_ros status=deprecated}



#### Controlling your robot with a joystick

SSH into the Raspberry Pi and run the following from the `duckietown` directory:


    duckiebot $ cd ~/duckietown
    duckiebot $ source environment.sh


The `environment.sh` setups the ROS environment at the terminal (so you can use
commands like `rosrun` and `roslaunch`).


Now make sure the motor shield is connected.

Run the command:


    duckiebot $ make demo-joystick


If there is no "red" output in the command line then pushing the left joystick
knob controls throttle - right controls steering.

Close the program using <kbd>Ctrl</kbd>-<kbd>C</kbd>.



#### Controlling your robot with the keyboard

ssh into your robot and run:


    duckiebot $ make demo-joystick


Now on your laptop run:


    laptop $ make virjoy-![Duckiebot name]


Note: you need to have `pygame` installed, see the [README](https://github.com/duckietown/Software/tree/master18/misc/virtualJoy)

Note: not currently supported on Mac OSX



It is possible you will have to unplug and replug the joystick or just push lots of buttons on your joystick until it wakes up. Also make sure that the mode switch on the top of your joystick is set to "X", not "D".

XXX Is all of the above valid with the new joystick?



## Troubleshooting 

Symptom: When do a `docker run` command you get a response like: `Error response from daemon: Get https://registry-1.docker.io/v2/: x509: certificate has expired or is not yet`

Resolution: you need to set the date on your robot to something close to reality. This can be done in an automated fashion (e.g. `ntpdate` ) but a bulletproof way is to run:


    duckiebot $ date -s "YYYY-MM-DD HH:MM:SS"



Symptom: The robot doesn't move when I use the joystick


Resolution: Check that the joystick receiver is in one of the USB port on the Raspberry Pi.

Resolution: Check that the joystick is detected by ssh'ing into the Duckiebot and running:

    duckiebot $ ls /dev/input/

and verify that  there is a device called `js0` on the list.

Resolution:  Test whether or not the joystick itself is working properly, run:

    duckiebot $ jstest /dev/input/js0

Move the joysticks and push the buttons. You should see the data displayed change
according to your actions.

Resolution: Check that the cables on your robot are connected 



Symptom: The robot moves weirdly (e.g. forward instead of backward).

Resolution: The cables are not correctly inserted. Please refer to [the assembly guide](#assembling-duckiebot-db18) for pictures of the correct connections. Try swapping cables until you obtain the expected behavior.

Resolution: Check that the joystick has the switch set to the position "x". And the mode light should be off.

Symptom: The left joystick does not work.

Resolution: If the green light on the right to the "mode" button is on, click the "mode" button to turn the light off. The "mode" button toggles between left joystick or the cross on the left.


<!--

## The proper shutdown procedure for the Raspberry Pi

Generally speaking, you can terminate any `roslaunch` command with <kbd>Ctrl</kbd>-<kbd>C</kbd>.

To completely shutdown the robot, issue the following command:

    duckiebot $ sudo shutdown -h now

Then wait 30 seconds.

Warning: If you disconnect the power before shutting down properly using `shutdown`,
the system might get corrupted.

Then, disconnect the power cable, at the **battery end**.

As an alternative you can use the `poweroff` command:

    duckiebot $ sudo poweroff

Warning: If you disconnect frequently the cable at the Raspberry Pi's end, you might damage the port.

-->

## Watch the program output using `rqt_console`

Also, you might have notice that the terminal where you launch the launch file
is not printing all the printouts like the previous example. This is one of
the limitation of remote launch.

Don't worry though, we can still see the printouts using `rqt_console`.

#### With Docker

On the laptop run:


    laptop $ dts start_gui_tools ![Duckiebot name]



#### With ROS

On the laptop, open a new terminal window, and run:


    laptop $ export ROS_MASTER_URI=http://![robot name].local:11311/
    laptop $ rqt_console


Either way, You should see a nice interface listing all the printouts in real time,
completed with filters that can help you find that message you are looking for
in a sea of messages.
If `rqt_console` does not show any message, check out the *Troubleshooting* section below.

You can use <kbd>Ctrl</kbd>-<kbd>C</kbd> at the terminal where `roslaunch` was executed to stop all the
nodes launched by the launch file.

See also: For more information about `rqt_console`, see [](+software_reference#rqt_console).

##### Troubleshooting

Symptom: `rqt_console` does not show any message.

Resolution: Open `rqt_console`. Go to the Setup window (top-right corner).
Change the "Rosout Topic" field from `/rosout_agg` to `/rosout`. Confirm.

Symptom: `roslaunch` fails with an error similar to the following:


    remote[![robot name].local-0]: failed to launch on ![robot name]:

    Unable to establish ssh connection to [![username]@![robot name].local:22]:
    Server u'![robot name].local' not found in known_hosts.


Resolution: You have not followed the instructions that told you to add the `HostKeyAlgorithms`
option. Delete `~/.ssh/known_hosts` and fix your configuration.

See: The procedure is documented in [](+software_reference#ssh-local-configuration).
