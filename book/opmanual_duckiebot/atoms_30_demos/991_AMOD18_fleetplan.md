# Fleet Management {#demo-fleetplan status=draft}


This is a demonstration for running the Fleet Management Simulator for comparing different Dispatchers (fleet planning algorithms). We focus our steps to run a baseline Dispatcher with the fplan simulator, which majorly consists of a dynamics engine of duckiebots and a visualization of their output in Rviz. The diagram below shows the architecture of the simulator software stack.

<br />
<figure>
<figcaption>Simulator Architecture</figcaption>
<img  style='width:24em'  src="Simulator_architecture.png"/>
</figure>
<br />

<div class='requirements' markdown="1">

Requires: [ROS-Desktop-Full][ROS-installation] installed

[ROS-installation]:http://wiki.ros.org/ROS/Installation

Requires: [duckietown-world][duckietown-world] installed

[duckietown-world]: https://github.com/duckietown/duckietown-world/
</div>

## Video of expected results {#demo-fleetplan-expected}

<div figure-id="fig:demo-fleet-management-simultator-video">
    <figcaption>Demo: Duckietown Fleet Management Simulator</figcaption>
    <dtvideo src='vimeo:306587628'/>
</div>

## Demo instructions {#demo-fleetplan-run}

**Step 1**: Download the `duckietown-fplan` respository

    laptop $ git clone --recursive https://github.com/duckietown/duckietown-fplan

**Step 2**: Build the `duckietown-fplan` respository

    laptop $ cd duckietown-fplan
    laptop $ catkin_make
 
 **Step 3**: Source the setup file (you can also add this step to your `.bashrc`)

    laptop $ source devel/setup.bash

 **Step 4**: To launch the simulator, visualization, and dummy planner demo
 
    laptop $ roslaunch flock_simulator flock_simulator.launch


`flock_simulator` receives commands on the topic `/flock_simulator/commands` and updates the state accordingly which is then published on `/flock_simulator/state`.

<br />
The duckiebots execute the commands (linear and angular velocity) that are tagged with their ID. If a duckiebot does not receive any commands, it drives around randomly (on rails). Once the duckiebot has received commands, it starts following them. The duckiebots stop when there are no more commands to execute. To fix slight deviations from "on rails" behavior, set the `on_rails` flag in the commands to `True` (will ignore if clearly not on rails).

<br />
The state is published as an array of `DuckieState`. The following information is contained:

```
duckie_id  # The ID of the duckiebot
on_service  # Boolean if a duckie has been picked up and is being dropped off, relevant for fleet planning
in_fov  # Array of duckiebot ids that are in its field of view
collision_level  # 0: no collision, 1: collision with duckiebot, 2: out of lane
pose  # Duckiebot's pose
velocity  # Duckiebot's velocity (currently set as the last command)
```

<br/>
The uncontrolled duckiebots will slow down if there is another duckiebot in front of them, traffic rules are yet to be implemented.

## Troubleshooting {#demo-fleetplan-troubleshooting}

While executing the  **Step 3** of the demo instructions, if you encounter issues related to the submodule `duckietown-visualization`, try the following steps.

<br />
Check if the submodule was downloaded

	laptop $ git submodule update --recursive --remote


Check if the submodule was updated

	laptop $ cd src/duckietown-visualization
	laptop $ git pull


Check if the submodule is on the correct branch

	laptop $ cd src/duckietown-visualization
	laptop $ git checkout visualization-fplan
