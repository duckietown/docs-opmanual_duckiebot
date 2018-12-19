# AMOD18 The Obstavoid Algorithm {#demo-theobstavoidalgorithm status=draft}

## Demo description {#demo-theobstavoidalgorithm-description}

This demo will show the use of a new fully-functional drive controller. It might replace the current lane follower in the future, as it is able to handle more general driving scenarios, mainly focusing on obstacles.

With the Obstavoid algorithm, the duckiebot (now denoted as the ‘actor’) will still try to follow its own lane, exactly like the lane follower implemented by default. The advantage of the Obstavoid algorithm is, that it takes into account every obstacle within a certain visibility range and planning horizon. The module continuously calculates an optimal trajectory to navigate along the road. If an obstacle blocks its way, the algorithm will decide on its own what maneuver will suit the best for the given situation. By predicting the motion of every obstacle and other duckiebots, the actor can, for example, be prepared for an inattentive duckie trying to cross the road, or decide if a passing maneuver is feasible with the current traffic situation, or if it should rather adjust its driving speed to wait until one of the lanes are free.

In this demo different scenarios are set up in a simulation, where you can test the Obstavoid algorithm to its full extent.


<div class='requirements' markdown="1">

Requires: Desktop-full installation of ROS - see [here](http://wiki.ros.org/kinetic/Installation)

Requires: ssh key for your computer - see [here](https://help.github.com/articles/adding-a-new-ssh-key-to-your-github-account/)

Requires: python 2.7 [here](https://www.python.org/download/releases/2.7/)

Requires: pip installation - see virtualenv [here](https://www.saltycrane.com/blog/2010/02/how-install-pip-ubuntu/)

Requires: installed catkin build [here](https://catkin-tools.readthedocs.io/en/latest/installing.html)

Requires: duckietown-world installation on local computer. [here](https://github.com/duckietown/duckietown-world)

Requires: Docker installation on local computer. TODO

</div>


Actor, passing a dynamic obstacle:
<div figure-id="fig:demo_1_no_cost_grid">
     <img src="994_AMOD18_TheObstavoidAlgorithm/demo_1_no_cost_grid.gif" style='width: 30em'/>
</div>


## The Obstavoid Algorithm {#demo-theobstavoidalgorithm-explained}

The Obstavoid algorithm is based on a shortest path optimisation problem, which seeks the best way through a weighted, three dimensional space-time grid. The three main pillars necessary for this problem setting are the design of a suitable cost function to define the actor’s behaviour, a graph search algorithm to determine the optimal trajectory and a sampler, which extracts the desired steering commands from a given trajectory and the actor’s position. In the following, each of these aspects will be further discussed and their implementation in the code architecture is briefly addressed.


Example 3D cost grid illustration:
<div figure-id="fig:cost_grid_diagram">
     <img src="994_AMOD18_TheObstavoidAlgorithm/cost_grid_diagram.jpeg" width=50%/>
</div>


### Cost function
There are three main goals the actor tries to fulfill when driving on the road. Firstly, the robot tries to stay within its own lane to obey traffic rules. Secondly, it wants to drive forwards, to not cause a traffic jam and reach its destination. Finally, the actor wants to avoid any collisions with obstacles, e.g. other duckies or duckiebots. For the algorithm to work, these three requirements need to be modelled as a cost function for the solver to find an optimal trajectory with minimal cost along its path.
For the actor to stay within its own lane the cost was shaped with a 5th degree polynomial curve with a global minimum in the center of the right lane. Driving in the wrong lane results in a higher cost as it is not desired but only needed for a passing maneuver or to dodge an obstacle. Getting closer to the edge of the road is punished with an even higher cost as an accident with an innocent duckie would be unforgivable.

By reducing the cost linearly along the driving direction of the road, it is beneficial for the actor to drive forwards. Both road cost and driving cost are independent of time, so these will remain the same for any given driving scenario.


Static cost function for lane following and forward motion:
<div figure-id="fig:cost_function_road_fwd">
     <img src="994_AMOD18_TheObstavoidAlgorithm/cost_function_road_fwd.png" width=50%/>
</div>


On the other hand, obstacles like other duckiebots cannot always be modelled statically. Consider a situation where the actor tries to drive around an obstacle, which is blocking the road. It needs to be sure, that no other duckiebot will come across the left lane while conducting a passing maneuver. To be able to predict the future within a certain time horizon, the cost grid needs to be extended to a third dimension, now considering the factor of time.

The costs of each obstacle is modelled as a modified, rotationally symmetrical bell curve. By considering the current velocity of a dynamic obstacle and assuming the speed and direction to remain constant, the position of the bell shaped cost function can be predicted for every time instance on the overall time-dependent cost function.


Modified bell curve cost function with tuning parameter to change the radial influence:
<div figure-id="fig:BellCurve">
     <img src="994_AMOD18_TheObstavoidAlgorithm/BellCurve.gif" width=50%/>
</div>


By simply adding these static costs with the cost of each obstacle together, an overall final cost is obtained. Once this final cost function has been found, it can now be discretized as a 5 x 6 x 6 grid (width x length x timesteps) in a defined box in front of the actor. The size of this grid has been evaluated for the solver to be able to compute the trajectory within a 10th of a second, allowing for a sampling frequency of 10 Hz.


Actor with visualisation of its cost grid. As the road is blocked, the lowest cost is reached by waiting:
<div figure-id="fig:demo_3_with_cost_grid">
     <img src="994_AMOD18_TheObstavoidAlgorithm/demo_3_with_cost_grid.gif" style='width: 30em'/>
</div>


### Solver
To determine an optimal trajectory through the time-dependent cost grid the problem can be rephrased as a shortest path problem [[2](http://www.idsc.ethz.ch/education/lectures/optimal-control.html)] in a three-dimensional volume.

Firstly, possible space-time positions (referred to as ‘nodes’) are connected using directional edges only, to ensure temporal causality. To minimize path search time and effort, edge connections are established only if they are physically feasible given the actor’s maximal velocity and sampling time of the discretization.

Secondly, in contrast to typical shortest path problems, not only the edge cost of traversing an edge between two nodes but also a node cost, which is the cost generated by visiting the corresponding node, has to be considered. As described in the previous section, the node costs are determined by an intricate cost function, whereby the edge costs are weighted proportionally to their lengths, leading to a slight preference for actually shorter paths.

As our selected discretization parameters lead to a manageable number of 180 nodes and ca. 1’500 edges, an optimal solution with no heuristics is feasible: The go-to choice to solve for this scenario is Dijkstra’s algorithm [[3](https://www.youtube.com/watch?v=GazC3A4OQTE)]. Not only does it guarantee the path of optimal cost [[4](https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm)], but it determines it with great efficiency. Emerging from the application of the dynamic programming algorithm on a general, shortest path problem, it only pursues to search paths while they are lower in cost than the previously best result. Finally, after applying Dijkstra’s algorithm to our problem, we obtain a time-stamped trajectory over the entire planning horizon.


A duckie crosses the road, but the Obstavoid Algorithm predicted its movement early enough:
<div figure-id="fig:demo_4_no_cost_grid">
     <img src="994_AMOD18_TheObstavoidAlgorithm/demo_4_no_cost_grid.gif" style='width: 30em'/>
</div>


### Trajectory sampler
Given this trajectory, the actor now needs to follow it as smoothly as possible. To get a continuous trajectory, the discretely evaluated six time-stamped positions obtained from the solver are linearized inbetween. At last, a trajectory follower calculates the linear and angular velocity commands to steer the robot.


Actor, following the lane:
<div figure-id="fig:demo_5_no_cost_grid">
     <img src="994_AMOD18_TheObstavoidAlgorithm/demo_5_no_cost_grid.gif" style='width: 30em'/>
</div>


### Optimality
What sets the Obstavoid Algorithm apart from a conventional trajectory optimizer is its inherent versatility: As the entire trajectory generation is one fluent process without any human-made case classifications, the transition between different driving maneuvers and actions happens without discrete switches. With this approach, a great variety of scenarios can be managed automatically, e.g. waiting for a passing maneuver while the other lane is blocked. This dynamic start-and-stop maneuvering strategy directly arises from the problem formulation, as it turns out that waiting before overtaking clearly has a lower cost than a crash or going back-and-forth.


The Actor is waiting until the other lane is free:
<div figure-id="fig:demo_2_no_cost_grid">
     <img src="994_AMOD18_TheObstavoidAlgorithm/demo_2_no_cost_grid.gif" style='width: 30em'/>
</div>


In conclusion, with the Obstavoid Algorithm the human influence on scenario analysis, classification and corresponding trajectory generation shifts towards the high-level tasks of cost-function design, which is finally not only a question of engineering but rather of morale and ethics.



### Software architecture

#### Description

Our pipeline is divided up into two main nodes which communicate via topic communication to the simulation (or duckiebot in the future).

**/trajectory_creator_node [frequency: 10hz]**
* **Input**: */flock_simulator/ state* and */flock_simulator/street_obstruction*: These topics contain position, veloccity and size infromation of all obstacles as well as information about the street the actor is at the moment. 
* **Computation**: 
**cost_grid_populator:** This manipulator uses the information safed in the obstacles and evaulates the cost function at the discretized points of the cost_grid.
**cost_grid_solver:** This manipulator finds an optimal path in the cost_grid while minimizing total cost.
* **Output:** */obst_avoid/trajectory*: This topic contains a target trajectory for the current cost_grid.

**/trajectory_sampler_node [frequency: 10 hz]**
* **Input**: */obst_avoid/trajectory*: This topic contains a target trajectory for the current cost_grid.
* **Computation**: 
**trajectory_sampler:** This manipulator uses the trajectory and derives the steering commands for the actor duckiebot.
* **Output:** */obst_avoid/trajectory*: This topic contains the current target linear and angular velocity of the bot.

#### Future Work

To improve the current pipeline on mulitple parts of the code coule be worked on:

* *test different cost_functions*: Currently the pipeline differentiates between static cost (given from things that cannot move) and dynamic cost (moving obstacles such as duckies and duckiebots). These functions could be changed and expanded easily in the 


<div figure-id="fig:software_architecture">
     <img src="994_AMOD18_TheObstavoidAlgorithm/software_architecture.png" style='width: 30em'/>
</div>

## Video of expected results {#demo-theobstavoidalgorithm-expected}

First, we show a video of the expected behavior (if the demo is succesful).


## Duckietown setup notes {#demo-theobstavoidalgorithm-duckietown-setup}

As this is only a demo in the simulation framework `duckietown-world`, no setup of a real duckietown is needed.


## Duckiebot setup notes {#demo-theobstavoidalgorithm-duckiebot-setup}

As this is only a demo in the simulation framework `duckietown-world`, no setup on a real duckiebot is needed.


## Pre-flight checklist {#demo-theobstavoidalgorithm-pre-flight}

Make sure you have a computer on which the following packages are installed (the installation has only been testes on Naitive Ubuntu 16.04)

* Check: Desktop-full installation of ROS - for instructions see [here](http://wiki.ros.org/kinetic/Installation)
* Check: that you have an ssh key for your computer [here](https://help.github.com/articles/adding-a-new-ssh-key-to-your-github-account/)
* Check: installed pip, virtualenv [here](https://www.saltycrane.com/blog/2010/02/how-install-pip-ubuntu/)
* Check: installed catkin build [here](https://catkin-tools.readthedocs.io/en/latest/installing.html)

TODO: make restart branch to master

## Demo instructions {#demo-theobstavoidalgorithm-run}

### Step 0 - 2 with no effort

If you are *lazy* you can just create a folder on your computer in which we will install everything you need for the demo and which you can easily delete afterwards. Just copy this file [here](https://raw.githubusercontent.com/duckietown/duckietown-mplan/restart/setup/setup_from_blank_folder.bash) in to the folder an run it in your folder with:

```
$ . ./setup_from_blank_folder.bash
```



### Step 1: Virtual environment setup

Make sure you have the prerequisites installed. We recommend running the whole setup in a virtual environment using `python2.7`.

create a virtual environment with
```
$ virtualenv -p python2.7 venv
```
activate the virtual environment
```
$ source venv/bin/activate
```

Install duckietown-world now in the virtual environment, as we depend on libraries from there - for instructions see [here](https://github.com/duckietown/duckietown-world)


### Step 2: Getting the mplan code

Clone the mplan-repo with the following command. Make sure you are inside the `src` folder of a catkin workspace. If you do not have a catkin workspace set up follow these [instructions](https://github.com/duckietown/duckietown-mplan/wiki/Setting-up-a-catkin-workspace).
```
$ git clone https://github.com/duckietown/duckietown-mplan.git
```

Enter the repo
```
$ cd duckietown-mplan
```

Install the additional requirements using
```
$ pip install -r requirements.txt
```

Load the submodules and build the workspace
```
$ git submodule init
$ git submodule update
```

Load the submodule’s submodules
```
$ cd duckietown-fplan
$ git submodule init
$ git submodule update
```
Build the workspace from the initial folder
```
$ cd ../../..
$ catkin build
```
Run `catkin_make` instead if you don't use `python-catkin-tools`.

Next, source your workspace using
```
$ source devel/setup.bash
```

### Step 3: Running the demo

Run the demo including a visualization in rviz with 
```
$ roslaunch obst_avoid obst_avoid_withviz_demo.launch demo_num:=1

```
With the parameter `demo_num` you can select a specific scenario. The scenarios are as follows:
 * 1 : dynamic passing of a moving object
 * 2 : passing of a static object with a dynamic object on the other side of the street
 * 3 : blocked road
 * 4 : street passing duckie
 * 5 : multiple obstacles and curves
  
### Step 4: Teleoperating another duckiebot (optional)


In the next step we will take control of another duckiebot, to see how the actor and the obstavoid algorithm will react to it.

Keep the simulation from before running and in a new terminal launch (don't forget to activate your virtualenvironment and source the catkin workspace)
```
$ rosrun obst_avoid duckiebot_teleop_node.py
```
Using 'i', 'j', 'l', ',' you can now teleoperate another duckiebot. With 'q', 'w' you can in-/ decrease it's speed. Make sure to keep the terminal selected, else the keyboard inputs will not be processed.


## Troubleshooting {#demo-theobstavoidalgorithm-troubleshooting}

 * 1 : Networkx library was not found: Double check that all installations were completed IN the virtual environment, especially the requirements of duckietown-world.
 * 2 : duckietown-world was not found: Double check that all installations were completed IN the virtual environment, especially the setup of duckietown-world.
 

## Demo failure demonstration {#demo-theobstavoidalgorithm-failure}

Our pipeline works with a fixed field of view, which can lead to situations where it is not able to react in time to avoid an obstacle if the obstacle comes to fast from the side.
<div figure-id="fig:fail_1_to_fast_from_side">
     <img src="994_AMOD18_TheObstavoidAlgorithm/fail_1_to_fast_from_side.gif" style='width: 30em'/>
</div>

For further analysis to the performance of the Obstavoid algorithm, refer to the next chapter.

## Performance analysis {#demo-theobstavoidalgorithm-performance}

%%%%%%%%%%%%%%%
dt ändere -> duckie schneller, tradeoff: geschwindigkeit - unvorsichtiger

measure execution time for different costs

cost grid verfiinere -> meh computation, smootheri trajektorie (screenshot)
cost grid meh time horizon -> bringt ned viel will mer ehh immer refreshed, aber schützt vor zuekünftigi ereignis (screenshot)


<div figure-id="fig:performance_graph">
     <img src="994_AMOD18_TheObstavoidAlgorithm/performance_graph.png" style='width: 30em'/>
</div>


## References {#demo-theobstavoidalgorithm-references}

%%%%%%%%%%%%%%%
LINK NETWORKX TO GRAPH


References:

* [1]: NetworkX documentation, a Python package for complex networks, see [here](https://networkx.github.io/) [Dec. 2018]

* [2]: Dynamic Programming and Optimal Control, lecture notes by Prof. Raffaello D'Andrea, see [here](http://www.idsc.ethz.ch/education/lectures/optimal-control.html) [Dec. 2018]

* [3]: Dijkstra's Algorithm, Video by Compterphile, see [here](https://www.youtube.com/watch?v=GazC3A4OQTE) [Dec. 2018]

* [4]: Dijkstra's Algorithm, Wikipedia, see [here](https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm) [Dec. 2018]


Further references, used in development:

* [5]: Graph algorithms, Wikipedia, see [here](https://en.wikipedia.org/wiki/Category:Graph_algorithms) [Dec. 2018]

* [6]: A* Algorithm, Video by Compterphile, see [here](https://www.youtube.com/watch?v=ySN5Wnu88nE) [Dec. 2018]






