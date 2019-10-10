# General Demo Running Procedure {#running-demos status=ready}

This page describes the basic procedure for running demos. Some demos have specific requirements that must be adhered to, but the general process of running them through the Duckietown shell is standardized.

<div class='requirements' markdown='1'>

Requires: A Duckiebot in `DB18` configuration that is [initalized](#setup-duckiebot)

Requires: Laptop configured, according to [](#laptop-setup).

Requires: Other requirements are demo specific, see the specific pages

</div>

## Start demos
In the [Duckietown software repo](https://github.com/duckietown/Software), there are two main types of launch files: node-specific (nuclear) ones, and demo launch files. A node launch file handles only a single node and its parameters. A demo launch file combines multiple of node launch files and adds the neccessary connections in order to stage demos that use dozens of nodes.

The launch procedure for both types is very similar. The generic command is:

    laptop $ dts duckiebot demo --duckiebot_name ![DUCKIEBOT_NAME] --demo_name ![DEMO_NAME] --package_name ![PACKAGE_NAME] --image duckietown/![IMAGE]:daffy
    
This command will start the `DEMO_NAME.launch` launch file in the `PACKAGE_NAME` package from the `duckietown/![IMAGE]:daffy` Docker image on the `DUCKIEBOT_NAME` Duckiebot.

Note: Currently `daffy` is the development branch and the `dts` commands work by default with the `master19` version. That is why you should __always__ specify the image with the `daffy` tag!

You can find the specific command for each demo in the corresponding part of the book. 


## Debug options
You can open a terminal in the container running the demo you want by appending the option `--debug` to the command. An example is:

    laptop $ dts duckiebot demo --duckiebot_name ![DUCKIEBOT_NAME] --demo_name ![DEMO_NAME] --package_name ![PACKAGE_NAME] --image duckietown/![IMAGE]:daffy --debug

This enables you to access to the `ROS` debug informations of the nodes that are launched. This is the same output that you can see in the `logs` window of the particular container on Portainer.
