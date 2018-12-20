# AMOD18 Planning Under Uncertainty {#demo-planningunderuncertainty status=draft}

**Planning under uncertainty** (observation uncertainty) is a simulation-based project for the AMOD-Duckietown Fall 2018 course at ETH Zurich.

First, we describe what is needed, including:

* Your laptop only


## Pre-flight checklist {#demo-planningunderuncertainty-pre-flight}



duckietown-world


Those following prerequisites ensure that the simulation will run properly:

Check: Desktop-Full installation of [ros-kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)

Check: [duckietown-world](https://github.com/duckietown/duckietown-world) (And all their requirements)

Check: Ubuntu16.04 with python2.7


## Video of expected results {#demo-planningunderuncertainty-expected}

<div figure-id="fig:simulation_uncertain">
    <figcaption>Simulation of the planning under uncertainity and the velocity profiler in action.
    </figcaption>
    <dtvideo src='vimeo:307404964'/>
</div>

## Duckietown setup notes {#demo-planningunderuncertainty-duckietown-setup}

Currently, plannning under uncertainity is tested only in simulation environments using rviz for visualization. In fact, the simulation works for any map type that is available (_Add  a new .yaml file in duckietown-world repo if a new map type is needed_). The package will be available as a ros package for testing on the actual duckiebots in a duckietown setup.


## Demo instructions {#demo-planningunderuncertainty-run}

Here, give step by step instructions to reproduce the demo.

Step 0: Make sure you sourced ros

    $ source /opt/ros/kinetic/setup.bash       (Use setup.zsh If you are using zsh shell)

Step 1: Create a catkin workspace in your home directory

    $ mkdir -p ~/catkin_ws/src
    $ cd ~/catkin_ws/src
  
Step 2: Clone the following repo [duckietown-uplan](https://github.com/duckietown/duckietown-uplan) 

    $ git clone https://github.com/duckietown/duckietown-uplan

Step 3: Installation of dependencies (NOTE: make sure that pip installation belongs to python2.7)

    $ cd duckietown-uplan/lib-uplan
    $ pip install -r --user requirements.txt        (Might require sudo or --user)
    $ sudo python setup.py develop --no-deps
    
Step 4: Go back to catkin_ws main directory and run catkin_make

    $ cd ..
    $ catkin_make
    
Ensure all three packages build correctly and custom messages are built

    $ source devel/setup.bash               (Or zsh)
    $ roslaunch uplan_visualization planningUncertainty.launch

## Experimentation

For experimentation. You can edit the following parameters and observe how the path plan and the velocities along the path change with it. (to be provided in a .yaml file)

* velocity min, velocity max, number of velocities for a duckiebot to choose from
* Graph augmentation parameters like number of extra lanes to the right, number of extra lanes to the left and number of control point to spawn between two original control points. Note: Augmenting the graph basically means how many paths the duckiebot will consider when planning under uncertainity. (The more you augment the graph, the slower the demo will be since there will be more control points to consider while planning)
* Duckie bot width and height
* Field of view of each duckiebot
* initial obstacle probability to consider (0 to 1)
* Duckietown map to use

## Features
* [x] Visualization of maps from duckietown-world
* [x] Planning under uncertainity to get a shortest collision free path under current observations.
* [x] Planning under uncertainity to get a velocity profiler for the shortest path chosen.
* [x] Realtime visualization of duckiebots
* [x] Visualization of 10 future steps of duckiebot trajectory
* [x] Visualizing uncertainty probability field
* [x] Visualizing velocity profiler
* [x] Visualizing of the shortest path planned




## Proposed Solution
We propose a planning algorithm that outputs a trajectory and corresponding velocity profile, in a discretized space, resembling a railway/subway planning to perform multi-objective optimization for the shortest path under uncertainty.
We predefine “legal” routes a duckiebot can take, including the ones at intersections. Each route is composed of several nodes and edges connecting them which are then further subdivided into a predefined number of line-segments. These predefined routes allow the duckiebot to go in a violating lane for example to pass another duckiebot or pass one.
Our proposed planner assigns a cost for each of the control points (nodes) taking into consideration the various degrees of uncertainty, and plan a path to goal state. A simple A* algorithm will be then used to define a velocity profile for the path calculated by the planner.
### Initialization:
All nodes are assigned a fixed value associated with the probability of an obstacle being present at that node.
The collision matrix defines the pairs of poses where the footprint of one duckiebot in one pose intersects that of another duckiebot in the other pose of that pair. Occupancy vector defines the set/vector of poses (graph nodes) that are occupied (through the footprint of the duckiebot).

### Planning:
Given a start and end goal the planner outputs a shortest path using Dijkstra's algorithm.
At each timestep the duckiebot updates its probability according to the obstacles it observes in its field view. Nodes with obstacles are assigned a cost of infinity cost and an uncertainty of 1. Obstacle free nodes are assigned a zero cost and zero uncertainty probability. 
Once the probabilities and collision matrices are updated in the ObservationModel, the planning algorithm replans a path to the goal.
The VelocityProfiler takes in this path and outputs a velocity profile for the given trajectory.
This process is continued until goal is reached.

### Visualization:
The trajectory output and the uncertainties calculated for the duckiebot are then published using markers and subscribes to by RViz. 
The trajectory path is colorized according to the velocity profile; high to low velocities are mapped from red to blue shades.
The uncertainties at each node are also colorized according to their magnitudes. Red shades represents higher probabilities of an obstacle being present and bluer shades represent unoccupied nodes.

### Results
* Goal 1: Obstacle avoidance (dynamic and static), i.e. collision-free shortest path
We can clearly see that duckiebots running the path planning algorithm successfully manage to avoid other duckiebots (be it stationary or dynamic).
For example, *something specific to the video*
* Goal 2: Velocity profiler 
The velocity profiling algorithm adds an additional measure of safety in that it limits the velocity of the duckiebot in regions where it is not so certain about its environment. Several factors are taken into account in the cost function (point out an example where different factors come into picture)
For example, *something specific to the video*

### Problems Faced/ Future Work
Computationally expensive to augment the graph (to be optimized in C++ library - 100x faster)


## More on Planning Under Uncertainty

### 1. Building of the augmentified pose graph network

The augmentified pose graph network is built from the original lane control points. It is the same graph as the one provided from the duckietown_world package but with more poses/ control points as well as other lanes; allowing the duckiebot to move more freely in the map to avoid obstacles and plan under uncertainity. The graph contains important information (SE(2) transforms, in-lane, distance-to-center, ...) that is used extensively in each of the submodules described next.

### 2. Collision-free shortest path planning

The collision matrix is built from the intersection of the possible footprints of the duckiebots in the map. (insert formulas here). The collision matrix is the same for a particular map and augmentified pose graph network; hence, it is calculated once at the beginning, and then saved to be used throughout the algorithm. Given an occupancy vector for all the nodes in the graph, one can figure out which set of nodes are forbidden by simply taking the product of the matrix and the occupancy vector.

The path planner takes into account both the shortest distance and blocked nodes, so that the end result is the shortest collision-free path. This path is sent as an output to the velocity profiling algorithm, described next.


### 3. Applying A* to the discretized velocity space

To plan a velocity trajectory using the path retrieved from the simulation we need to have a list of uncertainties of an obstacle being in the path and the list of the SE2Transforms objects:
* 1 means that we are certain that an obstacle is in a certain point in the path
* 0 means that we are completely certain that there is no obstacle

We apply A* algorithm to get the velocity profile for a given path. The cost function used here is:

For the planning algorithm to know how to plan for a good velocity profile we use a cost functions that includes the following terms:

* [delta_v_norm] Changes in velocity from node to node
* [delta_unc] The difference in uncertainty from node to node
* [unc] The uncertainty level
* [next_vel_norm] The next velocity normalized (v - v_min)/(v_max-v_min)
* [error_norm] Error from reference to actual speed (normalized) (reference is taken as max_v, this makes us go as fast as possible given that there is no uncertainty)
The current implemented cost function is:

    cost = a1*delta_v_norm^2 + a3*vel_norm*unc + a5*error_norm

Of course the cost function is customizable and can be change with a few lines of code, if no cost function is sent to Velocity profiler then it is going to use the above mentioned.

### 4. Experimenting with different cost functions


<div figure-id="fig:cost_func2" figure-caption="Cost Function 1">
     <img src="cost_func2.png" style='width: 30em'/>
</div>



Another example of a cost function:

<div figure-id="fig:cost_func3" figure-caption="Cost Function 2">
     <img src="cost_func3.png" style='width: 30em'/>
</div>


## More on Rviz visualizer

### 1.How it works

To understand the working of this package, you need a basic understanding of the
[ROS Transform library](http://wiki.ros.org/tf2) and 
[RViz Markers](http://wiki.ros.org/rviz/DisplayTypes/Marker). This package reads
the map having name `map_name` from `duckietown-world` and broadcasts markers
([MESH_RESOURCE](http://wiki.ros.org/rviz/DisplayTypes/Marker#Mesh_Resource_.28MESH_RESOURCE.3D10.29_.5B1.1.2B-.5D)) 
for each element of the map. Each class of elements (tiles, road signs, etc.) is 
published in a different namespace under the transform `/map` to provide the 
feature of turning off a certain category of map elements.
[ROS Custom Messages] The package uses two custom messages: duckieData.msg and DuckieStruct.msg
More information found here: http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv

### 2.Using uplan_visualization with your pipeline

Go through the publish_duckieData.py in the package ros-uplan folder to understand
how to use `uplan-visualization`. Here you will populate the duckie trajectory/velocity and uncertainty values and publish them as markers to RViz, using uncertainty_planning package.

* First, create a list of duckiebot names which you want to visualize in a yaml 
file similar to `uplan-visualization/config/example.yaml`
* For each duckiebot, publish a transform from frame `duckiebot_link` to that 
duckiebot. `duckiebot_link` is a fixed frame which is the origin of your 
measurements for each duckiebot.
* In the publish_trajec.py and publish_uncertainty.py ROS nodes, populate the duckieStruct meesages and publish on corresponding topics.
* The trajectory is plotted using the line marker_array and uncertainty using Cubelist marker_array.


## Demo failure demonstration {#demo-planningunderuncertainty-failure}

Video will be uploaded shortly....


## Results

Conclusion of the results and how uncertainities play a role for the velocity profiler



## Troubleshooting {#demo-planningunderuncertainty-troubleshooting}

Case 1: 

Case 2:




