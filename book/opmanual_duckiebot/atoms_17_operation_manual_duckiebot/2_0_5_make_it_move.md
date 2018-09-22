# Making Your Duckiebot Move {#rc-control status=ready}

Assigned: Breandan Considine, Liam Paull

This page is for the `DB18` configuration used in classes in 2018. For last year's instructions see [here](docs.duckietown.org/DT17/).

<div class='requirements' markdown='1'>


Requires: Laptop configured, according to [](#laptop-setup).

Requires: You have configured the Duckiebot. The procedure is documented in [](#setup-duckiebot).

Requires: You have created a Github account and configured public keys,
both for the laptop and for the Duckiebot. The procedure is documented in [](+software_reference#github-access).

Results: You can make your robot move.

</div>

## Option 1 - Pure Docker {#make-it-move_docker status=draft}



## Option 2 - Docker + ROS {#make-it-move_docker_ros status=draft}



## Option 3 - Pure ROS {#make-it-move_ros}



## Troubleshooting 

Add the date thing

## Test that the joystick is detected {#test-joystick}


Plug the joystick receiver in one of the USB port on the Raspberry Pi.

To make sure that the joystick is detected, run:

    duckiebot $ ls /dev/input/

and check if there is a device called `js0` on the list.

To test whether or not the joystick itself is working properly, run:

    duckiebot $ jstest /dev/input/js0

Move the joysticks and push the buttons. You should see the data displayed change
according to your actions.

## Run the joystick demo

SSH into the Raspberry Pi and run the following from the `duckietown` directory:

    duckiebot $ cd ~/duckietown
    duckiebot $ source environment.sh

<!-- duckiebot $ source set_ros_master.sh -->

The `environment.sh` setups the ROS environment at the terminal (so you can use
commands like `rosrun` and `roslaunch`).

<!-- The `set_ros_master.sh` script by default sets the Raspberry Pi as its own ROS master. -->

Now make sure the motor shield is connected.

Run the command:

    duckiebot $ roslaunch duckietown joystick.launch veh:=![robot name]

If there is no "red" output in the command line then pushing the left joystick
knob controls throttle - right controls steering.

This is the expected result of the commands:

```
<col2>
<span>left joystick up</span>     
<span>forward</span>
<span>left joystick down</span>   
<span>backward</span>
<span>right joystick left</span>  
<span>turn left (positive yaw)</span>
<span>right joystick right</span> 
<span>turn right (negative yaw)</span>
</col2>
```

It is possible you will have to unplug and replug the joystick or just push lots of buttons on your joystick until it wakes up. Also make sure that the mode switch on the top of your joystick is set to "X", not "D".

XXX Is all of the above valid with the new joystick?

Close the program using <kbd>Ctrl</kbd>-<kbd>C</kbd>.

### Troubleshooting

Symptom: The robot moves weirdly (e.g. forward instead of backward).

Resolution: The cables are not correctly inserted.
Please refer to the assembly guide for pictures of the correct connections.
Try swapping cables until you obtain the expected behavior.

Resolution: Check that the joystick has the switch set to the position "x".And the mode light should be off.

Symptom: The left joystick does not work.

Resolution: If the green light on the right to the "mode" button is on, click the "mode" button to turn the light off. The "mode" button toggles between left joystick or the cross on the left.

Symptom: The robot does not move at all.

Resolution: The cables are disconnected.

Resolution: The program assumes that the joystick is at `/dev/input/js0`.
In doubt, see [](#test-joystick).


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



## Watch the program output using `rqt_console`

Also, you might have notice that the terminal where you launch the launch file
is not printing all the printouts like the previous example. This is one of
the limitation of remote launch.

Don't worry though, we can still see the printouts using `rqt_console`.

On the laptop, open a new terminal window, and run:

```
laptop $ export ROS_MASTER_URI=http://![robot name].local:11311/
laptop $ rqt_console
```

You should see a nice interface listing all the printouts in real time,
completed with filters that can help you find that message you are looking for
in a sea of messages.
If `rqt_console` does not show any message, check out the *Troubleshooting* section below.

You can use <kbd>Ctrl</kbd>-<kbd>C</kbd> at the terminal where `roslaunch` was executed to stop all the
nodes launched by the launch file.

See also: For more information about `rqt_console`, see [](+software_reference#rqt_console).

## Troubleshooting

Symptom: `rqt_console` does not show any message.

Resolution: Open `rqt_console`. Go to the Setup window (top-right corner).
Change the "Rosout Topic" field from `/rosout_agg` to `/rosout`. Confirm.



Symptom: `roslaunch` fails with an error similar to the following:

```
remote[![robot name].local-0]: failed to launch on ![robot name]:

Unable to establish ssh connection to [![username]@![robot name].local:22]:
Server u'![robot name].local' not found in known_hosts.
```

Resolution: You have not followed the instructions that told you to add the `HostKeyAlgorithms`
option. Delete `~/.ssh/known_hosts` and fix your configuration.

See: The procedure is documented in [](+software_reference#ssh-local-configuration).