# Reading from the camera {#read-camera-data status=beta}


<div class='requirements' markdown='1'>

Requires: You have configured the Duckiebot.
The procedure is documented in [](+opmanual_duckiebot#setup-duckiebot).

Requires: You know [the basics of ROS](+software_reference#introduction_to_ros) (launch files, `roslaunch`, topics, `rostopic`).

Results: You know that the camera works under ROS.

</div>


## Check the camera hardware

It might be useful to do a quick camera hardware check.

See: The procedure is documented in [](#camera-hardware-check).

## Create two windows

On the laptop, create two Byobu windows.

See: A quick reference about Byobu commands is in [](+software_reference#byobu).

You will use the two windows as follows:

- In the first window, you will launch the nodes that control the camera.
- In the second window, you will launch programs to monitor the data flow.


Note: You could also use multiple *terminals* instead of one terminal with multiple Byobu
windows. However, using Byobu is the best practice to learn.

## First window: launch the camera nodes

In the first window, we will launch the nodes that control the camera.
All the following commands should be run in the `~/duckietown` directory:

    duckiebot $ cd ~/duckietown

Activate ROS:

    duckiebot $ source environment.sh

Run the launch file called `camera.launch`:

    duckiebot $ roslaunch duckietown camera.launch veh:=![robot name]

At this point, you should see the red LED on the camera light up continuously.

In the terminal you should not see any red message, but only happy messages like the following:

    ![...]
    [INFO] [1502539383.948237]: [/![robot name]/camera_node] Initialized.
    [INFO] [1502539383.951123]: [/![robot name]/camera_node] Start capturing.
    [INFO] [1502539384.040615]: [/![robot name]/camera_node] Published the first image.


See also: For more information about `roslaunch` and "launch files", see [](+software_reference#roslaunch).

## Second window: view published topics

Switch to the second window.
All the following commands should be run in the `~/duckietown` directory:

    duckiebot $ cd ~/duckietown

Activate the ROS environment:

    duckiebot $ source environment.sh

### List topics

You can see a list of published topics with the command:

    duckiebot $ rostopic list

See also: For more information about `rostopic`, see [](+software_reference#rostopic).

You should see the following topics:

    /![robot name]/camera_node/camera_info
    /![robot name]/camera_node/image/compressed
    /![robot name]/camera_node/image/raw
    /rosout
    /rosout_agg

### Show topics frequency

You can use `rostopic hz` to see the statistics about the publishing frequency:

    duckiebot $ rostopic hz /![robot name]/camera_node/image/compressed

On a Raspberry Pi 3, you should see a number close to 30 Hz:

    average rate: 30.016
        min: 0.026s max: 0.045s std dev: 0.00190s window: 841

### Show topics data

You can view the messages in real time with the command `rostopic echo`:

    duckiebot $ rostopic echo /![robot name]/camera_node/image/compressed

You should see a large sequence of numbers being printed to your terminal.

That's the "image" --- as seen by a machine.

If you are Neo, then this already makes sense. If you are not Neo, in
[](+opmanual_duckiebot#rc-cam-launched-remotely), you will learn how to visualize the image stream
on the laptop  using `rviz`.


use <kbd>Ctrl</kbd>-<kbd>C</kbd> to stop `rostopic`.


TODO: Physically focus the camera.
