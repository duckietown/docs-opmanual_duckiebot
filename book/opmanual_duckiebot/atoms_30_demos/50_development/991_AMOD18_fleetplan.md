# AMOD18 Fleet Management {#demo-fleetplan status=draft}


This is a demonstration for running the Duckietown Fleet Management Simulator for comparing different dispatching algorithms in real time. We focus our steps to run a baseline dispatcher with the fplan simulator, which majorly consists of a dynamics engine of duckiebots and a visualization of their output in Rviz.
<br />

The Github repository for this demo can be found on https://github.com/duckietown/duckietown-fplan.
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
    <dtvideo src='vimeo:307327906'/>
</div>

## Demo setup notes

The fleetplan demo is proposed in the simulation environment using Rviz as a visualiser.


### Available Maps

Simple tile map (Default): `4way`

<br />
<figure>
<figcaption>Simple 4 way tile map</figcaption>
<img  style='width:12em'  src="f4way.png"/>
</figure>

<br />
ROBOTARIUM map: `robotarium1`

<figure>
<figcaption>Robotarium map</figcaption>
<img  style='width:12em'  src="robotarium1.png"/>
</figure>
<br />

### Fleet size
Any integer from `1 to 15`, the default value is set to `3` (Any higher integer is not recommended due to the limited available space on the available maps)

### Request arrival rate (in seconds)
Any reasonable integer, the default value is set to `20`.


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


## "Try it yourself" (Advanced)

If you think you can beat our proposed dispatching baseline algorithm, try it out yourself.

<br />
**Step 5**: Open the currently available dispatcher with the file reader of your choice

    laptop $ gedit src/flock_planner/src/dispatcher.py

The dispatcher takes in as input the `state` of the simulator and outputs the `paths` for each duckiebot.

<br />
The state is published and build as follows:

```
state = {
    'duckies': {
        'duckie-0': {
            'status': 'IDLE',
            'lane': 'l001',
        },
        'duckie-1': {
            'status': 'IDLE',
            'lane': 'l042',
        }, ...
    },
    'requests': {
        'request-0': {
            'time': 63,  # Timestep of request
            'duckie_id':
            'duckie-2',  # Duckie which is serving the request (empty if unassigned)
            'start_node': 'P13',  # Start node of graph (networkx)
            'end_node': 'P02',  # End node of graph
        }, ...
    }
}
```
<br />

<br />
The dispatcher should return `paths` for the single duckie bots as follows:

```
paths = [
    {
        'duckie_id': 'duckie-0',
        'request_id': 'request-2',
        'path': ['P03', 'P12', 'P08']
    },
    {
        'duckie_id': 'duckie-1',
        'request_id': None,  # Puts duckie into rebalancing mode
        'path': ['P03', 'P06']
    },
    ...
]
```

- If a duckie does not receive any commands, it drives around randomly with status 'IDLE' and does not pick up any request.
- The duckie has the status 'DRIVINGTOCUSTOMER' or 'DRIVINGWITHCUSTOMER' if a request_id is given. A path is needed, otherwise the duckie drives around randomly and picks up/drops off the request if it drives by by chance.
- If a path is given and no request_id, the duckie will follow the path and take the status 'REBALANCING'.

The map is given as a class variable `skeleton_graph` from the `duckietown_world` package. It is documented there in more detail if more information is needed.

`skeleton_graph.G` is a MultiDiGraph from the NetworkX library. Every node represents a fork in the path (i.e. before an intersection), or the merging of paths (after an intersection). The path will be given as a list of nodes, which the shortest one can be easily extracted with the NetworkX library (the shortest might not be the fastest path!). The edges contain the names of the lane it represents (the state gives you the duckie's position as their lanes their currently on).

`skeleton_graph.root2` contains a dictionary of poses that define the geometry for each lane. This information might be used for instance for generating weights for the lanes (e.g. length of lane), but is not necessary to create a working dispatcher.

<br/>


 **Step 6**: Launch the simulator and the visualization

    laptop $ roslaunch flock_simulator flock_simulator.launch

Note: to change the map name append to the command `map_name:="robotarium1"`, to change the number of requests append  `n_requests:="k"` and to change the arrival rate of request append  `t_requests:="l"`


## Simulator Architechture

The diagram below shows the architecture of the simulator software stack.

<br />
<figure>
<figcaption>Simulator Architecture</figcaption>
<img  style='width:24em'  src="Simulator_architecture.png"/>
</figure>
<br />

The simulator represents duckies as instances of the class `Duckiebot` and requests as instances of the class `Request`. A single instance of class `StateManager` is used to represent the entire state of the simulation and coordinate the interaction between duckies, requests and the map. The class `DuckietownMap` contains relevant information of the map and methods for extracting information.

### State
The state of the simulator is published on `/flock_simulator/state` with every update (triggered by messages on `/flock_simulator/commands`). The message contains following information:
```
std_msgs/Header header  # Generic header
flock_simulator/DuckieState[] duckie_states  # An array of duckie states
flock_simulator/Request[] requests  # An array of requests that have not been filled (either waiting or being driven around)
flock_simulator/Request[] filled_requests  # An array of filled requests (picked up and dropped off again)
```

#### Duckie state
The duckie state message contains following information:
```
std_msgs/String duckie_id  # The duckie's ID
std_msgs/String[] in_fov  # An array of all the duckies that it can see in its field of view
std_msgs/String status  # For fleetplanning ('IDLE', 'REBALANCING', 'DRIVINGTOCUSTOMER', 'DRIVINGWITHCUSTOMER')
std_msgs/String lane  # Current lane the duckie is on
std_msgs/UInt8 collision_level  # 0: No collision, 1: collision
geometry_msgs/Pose2D pose  # Current pose
geometry_msgs/Twist velocity  # Current velocity
```

#### Requests
A request message is defined by:
```
std_msgs/String request_id  # The request ID
std_msgs/UInt32 start_time  # The timestep of the request creation (calculate with dt to get time in seconds)
std_msgs/UInt32 pickup_time  # The timestep of pickup (0 if not picked up)
std_msgs/UInt32 end_time  # The timestep of drop-off (0 if not dropped off)
std_msgs/String start_node  # The node for pick-up (see _Map_ later)
std_msgs/String end_node  # The node for drop-off
std_msgs/String duckie_id  # The duckie that picked up the request (empty if request is waiting)
```

### Commands
A command can be issued on the topic `/flock_simulator/commands`, which also triggers a timestep for the simulation (the simulator does not do anything if there are no commands). A command for the flock has the following structure:
```
std_msgs/Header header  # Generic header
std_msgs/Float64 dt  # The timestep duration of the simulation in seconds, should not be too big (ideally ~0.05 or less)
flock_simulator/DuckieCommand[] duckie_commands  # An array for individual commands for duckies
```

#### Duckie command
The command message for a single duckie is defined as follows:
```
std_msgs/String duckie_id  # The duckie this command is meant for
std_msgs/Bool on_rails  # Set to True if the command is defined by the path below, set to False if the command is given as velocities in velocity_command below
std_msgs/String[] path  # An array of nodes that define the path
std_msgs/String request_id  # The request ID the duckie should serve (picks up or drops off if driving by)
geometry_msgs/Twist velocity_command  # Velocity commands (linear velocity in linear.x, angular velocity in angular.z)
```
Commands can be given as a path or as velocites. For the former, `on_rails` should be set to `True` in which case the duckie will ignore `velocity_command`, and vice versa.

The "on-rails" duckies follow very simple traffic rules, such as keeping a certain distance from duckies in front and giving right of way to duckies approaching intersections from the right. Also, they stop in front of intersections if there is already a duckie on it. **Note:** The current rules occasionally lead to collisions.

### Visualization
The simulation is visualized using rviz. The duckiebots are shown as meshes of their real-life counterparts, requests shown as duckies and drop-off points as green spheres. The duckiebots have a box on top taking on the color of their respective status (blue for `IDLE`, green for `REBALANCING`, yellow for `DRIVINGTOCUSTOMER` and orange for `DRIVINGWITHCUSTOMER`).

### Flock planner
The flock planner interacts with the simulator and publishes commands on it.

#### Dispatcher
The dispatcher receives the state of the simulation and returns paths for each duckie. The currenlty implemented dispatcher is deliberately kept very rudimentary, so it can be improved by users (and in the future hopefully AIDO participants) and tested.

## Troubleshooting {#demo-fleetplan-troubleshooting}

Symptoms: Collision between duckiebots `i` and `j`

```
duckie-i collided!
duckie-j collided!
```

<br />
Resolution:
<br />
Quit with <kbd>Ctrl</kbd>+<kbd>c</kbd> your terminal session and restart the demo at **Step 4**

<br />
Symptoms: While executing the  **Step 3** of the demo instructions, if you encounter issues related to the submodule `duckietown-visualization`, try the following steps.

<br />
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
