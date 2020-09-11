# Using start_gui_tools, ROS tools and no-vnc {#using-no-vnc status=ready}

## A note on start_gui_tools {#start_gui_tools status=ready}

If you are very familiar with how ROS works, and is familiar with command line tools, you can simply run:

    laptop $ dts start_gui_tools ![DUCKIEBOT_NAME]

This will give you a terminal (container actually) that is connected to the duckiebot ROS network, and you can perform all the ROS commands there too. 

Note: You can only start one instance of `start_gui_tools` container, therefore if you want multiple instances of terminal, it is recommended to use no-vnc.

Note: A trick you can use is to add `AND Symbol` at the end of your command to run it back ground. 

## Starting no-vnc images {#starting-no-vnc status=ready}

To start a image that runs no-vnc, do:

    laptop $ dts start_gui_tools --vnc ![DUCKIEBOT_NAME]

To use no-vnc, use your browser and navigate to:

    http://localhost:8087/

Once logged in, you can treat this environment as a typical Ubuntu machine with ROS installed, and configured to talk with your duckiebot.

## Verifying the output by using the ROS utilities and command line {#no-vnc-ros status=ready}

Open up a terminal (LXTerminal on the desktop), and use the commands below to check the data streams in ROS.

### List topics

You can see a list of published topics with the command:

    container $ rostopic list

See also: For more information about `rostopic`, see [](+software_reference#rostopic).

You should see at least the following topics:

    /![hostname]/camera_node/camera_info
    /![hostname]/camera_node/image/compressed
    /rosout
    /rosout_agg

There might be other topics if you started other demos.

### Show topics frequency

You can use `rostopic hz` to see the statistics about the publishing frequency:

    container $ rostopic hz /![hostname]/camera_node/image/compressed

On a Raspberry Pi 3, you should see a number close to 30 Hz:

    average rate: 30.016
        min: 0.026s max: 0.045s std dev: 0.00190s window: 841

Use CTRL-C to stop `rostopic`.

### Show topics data

You can view the messages in real time with the command `rostopic echo`:

    container $ rostopic echo /![hostname]/camera_node/image/compressed

You should see a large sequence of numbers being printed to your terminal.

That's the "image" --- as seen by a machine.

## rqt_image_view tool {#rqt-no-vnc status=ready}

To see what your duckiebot sees, you can click on the RQT Image View application icon on the desktop. You will see the `rqt_image_view` starts up

<figure>
    <figcaption>The rqt image view window with dropdown menu</figcaption>
    <img style='width:12em' src="rqt_image_view.png"/>
</figure>

## rqt_graph tool {#rqt-graph-no-vnc status=ready}

If you want to explore the relationship between all the nodes, topics and tf, you can open up a terminal and run:

    $ rqt_graph

This will open up a window that contains all the ROS topics being published, all the ROS nodes running, and it is a very handy tool to understand the relationship between nodes.

## ROS Troubleshooting

Symptom: My ros commands are not working. I cannot use tab to auto complete ROS commands. 

Resolution: You can fix that by sourcing devel/setup.bash.

    $ source /code/catkin_ws/devel/setup.bash

Symptom: I cannot connect to ROS master. 

Resolution: Go to protainer of the robot to make sure the following containers are running without errors: 

    ROS
    car-interface
    duckiebot-interface

If they are not running, refer to [docker troubleshooting](#setup-troubleshooting-docker) to make sure those containers are running.