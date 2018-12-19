# AMOD18 Fleet Management {#demo-fleetplan status=draft}


This is a demonstration for running the Robotarium Fleet Management Simulator for comparing different dispatching algoritms in real time. We focus our steps to run a baseline dispatcher with the fplan simulator, which majorly consists of a dynamics engine of duckiebots and a visualization of their output in Rviz.


<div class='requirements' markdown="1">

Requires: [ROS-Desktop-Full][ROS-installation] installed

[ROS-installation]:http://wiki.ros.org/ROS/Installation

Requires: [duckietown-world][duckietown-world] installed

[duckietown-world]: https://github.com/duckietown/duckietown-world/

</div>

## Video of expected results {#demo-fleetplan-expected}

<div figure-id="fig:demo-fleet-management-simultator-video">
    <figcaption>Demo: Duckietown Robotatarium Fleet Management Simulator</figcaption>
    <dtvideo src='vimeo:306587628'/>
</div>

## Simulator Architechture

The diagram below shows the architecture of the simulator software stack.

<br />
<figure>
<figcaption>Simulator Architecture</figcaption>
<img  style='width:24em'  src="Simulator_architecture.png"/>
</figure>
<br />

## Demo setup notes 

The fleetplan demo is proposed in the simulation environment using Rviz for visualization.

Tunable metrics:

* Available Maps

    - `4way` Example tile map (Default) **TODO** ADD PHOTO
    - `robotarium1` ROBOTATIUM map **TODO** ADD PHOTO
    - `udem1`

* Fleet size
	- Any integer from `1` to `15` (Any higher integer is not recommended due to the limited available space on the available maps) (default: `3`)

* Request arrival rate (in seconds)
	- Any reasonable integer. (default: `20`)


## Demo instructions {#demo-fleetplan-run}

Running the baseline.

**Step 1**: Download the `duckietown-fplan` repository

    laptop $ git clone --recursive https://github.com/duckietown/duckietown-fplan

**Step 2**: Build the `duckietown-fplan` repository

    laptop $ cd duckietown-fplan
    laptop $ catkin_make
 
 **Step 3**: Source the setup file (you can also add this step to your `.bashrc`)

    laptop $ source devel/setup.bash

 **Step 4**: To launch the simulator, visualization, and dummy planner demo
 
    laptop $ roslaunch flock_simulator flock_simulator.launch

 **Step 5**: Checkout the scores in **TODO**
 

## "Try it yourself" (Advanced) 

If you think you can beat our proposed dispatching baseline algorithm, try it out yourself. 

 **Step 6**: Open the currently available dispatcher with the file reader of your choice 

    laptop $ gedit src/flock_planner/src/dispatcher.py

The dispatcher takes in as input the `state` of the simulator and outputs the `paths` for each duckiebot.

<br />
The state is published and build as follows:

```
state = {
   'duckies': [
       'duckie-0': {
           'status': 'IDLE',
           'lane': 'l001',
       },
       'duckie-1': {
           'status': 'IDLE',
           'lane': 'l042',
       }, ...
   ]
   'requests': [
       'request-0': {
           'time': [time of request],
           'duckie_id': [duckie which is serving the request (empty if unassigned)],
           'start_node': [start node of graph (networkx)],
           'end_node': [end node of graph (networkx)],
       }, ...
   ]
 }
```
<br />

<br />
The dispatcher should return `commands` for the single duckie bots as follows: 

```
 paths = [
   {
       'duckie_id': 'duckie-0',
       'request_index': 2,
       'path': [list of nodes]
   },
   {
       'duckie_id': 'duckie-1',
       'request_index': 0,
       'path': [list of nodes]
   }, ...
 ]
```

<br/>


 **Step 7**: Launch the simulator and the visualization

    laptop $ roslaunch flock_simulator flock_simulator.launch
    
Note: to change the map name append to the command `map_name:="small_loop"`, to change the number of requests k `n_requests:="k"` and to change the arrival rate of request l `t_requests:="l"`

 **Step 8**: Checkout your scores in **TODO** and compare with our baseline. 

## Software architecture of the simulator


The `flock_simulator` receives commands on the topic `/flock_simulator/commands` and updates the state accordingly which is then published on `/flock_simulator/state`.

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
The uncontrolled duckiebots will slow down if there is another duckiebot in front of them, other traffic rules are yet to be implemented.

## Troubleshooting {#demo-fleetplan-troubleshooting}

Symptoms: Collision between duckiebots `i` and `j`

```
duckie-i collided!
duckie-j collided!
```

Resolution: Quit with `Ctrl` + `c` your terminal session and restart the demo at **Step 4**

Symptoms: While executing the  **Step 3** of the demo instructions, if you encounter issues related to the submodule `duckietown-visualization`, try the following steps.

Resolution:
<br />
Check if the submodule was downloaded

	laptop $ git submodule update --recursive --remote


Check if the submodule was updated

	laptop $ cd src/duckietown-visualization
	laptop $ git pull


Check if the submodule is on the correct branch

	laptop $ cd src/duckietown-visualization
	laptop $ git checkout visualization-fplan




## The AMOD18 Fleet Planning Report

### Mission and Scope

The goal of this project is to allow a real time visualization of implemented dispatching and rebalancing policies for the fleet planning of Autonomous Duckies on Demand (ADoD). 

### Motivation

#### Existing solution

#### Opportunity 

### Problem Definition

#### Final objective

### Added Functionalities

#### Fleet Planner
