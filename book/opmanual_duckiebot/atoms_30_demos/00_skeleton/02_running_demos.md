# General Demo Running Procedure {#running-demos status=ready}

This page describes the basic procedure for running demos. Some demos have specific requirements that must be adhered to, but the general process of running them through the Duckietown shell is standardized.

<div class='requirements' markdown='1'>

Requires: A Duckiebot in `DB18` configuration that is [initalized](#setup-duckiebot) has [camera](#camera-calib) and [wheels](#wheel-calibration) calibrated

Requires: Laptop configured, according to [](#laptop-setup).

Requires: Other requirements are demo specific, see the specific pages

</div>


In the [Duckietown software repo](https://github.com/duckietown/Software), the launch files are currently separated in two different packages (folders).

**The [`duckietown` package](https://github.com/duckietown/Software/tree/master19/catkin_ws/src/00-infrastructure/duckietown/launch)** has launch files which are constructed "additively" through `include` tags. This is the default package used to run launch files. Any launch file in this folder can be run through the [duckietown shell](#laptop-setup-ubuntu-18-shell) with the following command:

    laptop $ dts duckiebot demo --duckiebot_name ![DUCKIEBOT_NAME] --demo_name ![DEMO_NAME]

where `![DEMO_NAME]` is the part before the `.launch` of a `![DEMO_NAME].launch` file.

**The [`duckietown_demos` package](https://github.com/duckietown/Software/tree/master19/catkin_ws/src/70-convenience-packages/duckietown_demos/launch)** contains generally more complicated assemblies of capabilities that are composed into actions. These launch files are constructed "destructively" where each one includes the [`master.launch`](https://github.com/duckietown/Software/blob/master19/catkin_ws/src/70-convenience-packages/duckietown_demos/launch/master.launch) which contains `include` blocks for every node that exists in the Duckietown Software repository. These include blocks are activated and de-activated through a series of `args` that act as switches which are structured hierarchically at the top of the `master.launch` file. To run any launch file in this package through the shell (and actually any launch file in any package) you can additionally specify the `package_name` as an argument:

    laptop $ dts duckiebot demo --duckiebot_name ![DUCKIEBOT_NAME] --demo_name ![DEMO_NAME] --package_name duckietown_demos

where, similarly to above,  `![DEMO_NAME]` is the part before the `.launch` of a `![DEMO_NAME].launch` file.
