# Using no-vnc and ROS {#using-no-vnc status=ready}

## Starting no-vnc images {#starting-no-vnc status=ready}

To start a image that runs no-vnc, do:

    laptop $ dts start_gui_tools ![DUCKIEBOT_NAME]

To use no-vnc, use your browser and navigate to:

    http://localhost:6901/vnc.html. 
    
Password is quackquack.

Once logged in, you can treat this environment as a typical Ubuntu machine with ROS installed, and configured to talk with your duckiebot.

## Verifying the output by using the ROS utilities {#no-vnc-ros status=ready}

Open up a terminal (right click and select open terminal here), and use the commands below to check the data streams in ROS.

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

To see what your duckiebot sees, you can run:
    
    $ dts start_gui_tools ![DUCKIEBOT_NAME]