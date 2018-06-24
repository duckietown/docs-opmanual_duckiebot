# RC control launched remotely {#rc-launched-remotely status=beta}

Assigned: Andrea Censi

<div class='requirements' markdown='1'>

Requires: You can run the joystick demo from the Raspberry Pi.
The procedure is documented in [](#rc-control).

Results: You can run the joystick demo from your laptop.

</div>

## Two ways to launch a program

ROS nodes can be launched in two ways:

1. "local launch": log in to the Raspberry Pi using SSH and run
   the program from there.
2. "remote launch": run the program directly from a laptop.

Which is better when is a long discussion that will be done later.
Here we set up the "remote launch".

TODO for Andrea Censi: draw diagrams

## Download and setup `Software` repository on the laptop

As you did on the Duckiebot, you should clone the `Software`
repository in the `~/duckietown` directory.

See: The procedure is documented in [](#clone-software-repo).

Then, you should build the repository.

See: This procedure is documented in [](#build-repo).

## Rebuild the machines files {#create-robot-config}

In a previous step you have created a robot configuration file and pushed it to the duckiefleet repo. Now you have to pull duckiefleet on the laptop and rebuild the machines configuration file there.

See: The procedure is documented in [](#edit-machines-file).

## Start the demo {#roslaunch-joystick}

Now you are ready to launch the joystick demo remotely.

<div class='check' markdown='1'>

Make sure that you can login with SSH **without a password**.
From the laptop, run:

    laptop $ ssh ![username]@![robot name].local

If this doesn't work, you missed some previous steps.

</div>


Run this *on the laptop*:

    laptop $ source environment.sh
    laptop $ roslaunch duckietown joystick.launch veh:=![robot name]

You should be able to drive the vehicle with joystick just like the last
example. Note that remotely launching nodes from your laptop doesn't mean that
the nodes are running on your laptop. They are still running on the Raspberry Pi in this
case.

See also: For more information about `roslaunch`, see [](+software_reference#roslaunch).

## Watch the program output using `rqt_console`

Also, you might have notice that the terminal where you launch the launch file
is not printing all the printouts like the previous example. This is one of
the limitation of remote launch.

Don't worry though, we can still see the printouts using `rqt_console`.

On the laptop, open a new terminal window, and run:

    laptop $ export ROS_MASTER_URI=http://![robot name].local:11311/
    laptop $ rqt_console

You should see a nice interface listing all the printouts in real time,
completed with filters that can help you find that message you are looking for
in a sea of messages.
If `rqt_console` does not show any message, check out the *Troubleshooting* section below.

You can use <kbd>Ctrl</kbd>-<kbd>C</kbd> at the terminal where `roslaunch` was executed to stop all the
nodes launched by the launch file.

See also: For more information about `rqt_console`, see [](#rqt_console).


## Troubleshooting

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
