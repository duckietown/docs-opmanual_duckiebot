# General Exercise Running Procedure {#running-exercies status=ready}

TODO: Liam to add video

This page describes the exercises infrastructure. This infrastructure affords a seamless method to build on existing baselines, test them in simulation, test them on robot hardware either remotely or locally, and then evaluate and submit them as challenges with the [AIDO challenges infrastructure](+AIDO#book).

<div class='requirements' markdown='1'>

Requires: A Duckiebot  that is initalized

Requires: Laptop configured, according to [](#laptop-setup).

Requires: That you are able to to submit a challenge according to [](+AIDO#cm-first). 


</div>


## Getting Started


Fork the [`dt-exercices`](https://github.com/duckietown/dt-exercises) repo and clone it onto your computer. 
Set up an upstream remote. From inside the directory you just cloned:

    laptop $ git remote add upstream git@github.com:duckietown/dt-exercises.git

Now to pull anything new from the original repo you can do:

    laptop $ git pull upstream daffy

Enter the `dt-exercises` folder that you just cloned do:
 
    laptop $ cd dt-exercises

In here you will see a number of foldrers. Each folder corresponds to an exercise. 
It is recommended that you install the requirements (this can be in a `virtualenv` if you prefer of course):

    laptop $ pip install -r requirements.txt

    
    
## The Anatomy of an Exercise


Exercises should contain all of the following:

### `config.yaml`

This contains information about the exercise. Example: 

```yml
# Exercise configuration file

agent_base: "duckietown_baseline" # currently not implemented, the agent base image to use
ws_dir: "exercise_ws" # directory that contains the code
agent_run_cmd: "run_all.sh" # the script in "launchers" to run the agent with
```

TODO: Vincenzo add the configuration for notebooks. 

The `agent_base` indicates which image to use as a baseline to build from. The mappings are listed [here](https://github.com/duckietown/duckietown-shell-commands/blob/daffy/utils/exercise_utils.py). Many of the existing exercises are build on the `duckietown_baseline` image which contains all of the code in the [`dt-core` repository](https://github.com/duckietown/dt-core). 

Note: In the case that you are using the `duckietown_baseline`, any package/node that you create in the `exercise_ws` directory will be run instead of the one in the [`dt-core` repository](https://github.com/duckietown/dt-core) if the package name and node name match. This is achieved through [workspace overlaying](http://wiki.ros.org/catkin/Tutorials/workspace_overlaying). 

The `ws_dir` indicates the name of the subdirectory that contains the code that should be mounted into the image. 

The `agent_run_cmd` indicates the command that should be run when the container is run to start things. 


### `exercise_ws`

As indicated above, the `exercise_ws` directory is where the code should go. For the case of ROS packages, they should go inside a `src` directory inside `exercise_ws`. 

### `assets`

The `assets` folder contains two subfolders, `setup` and `calibrations`.

 - The `setup` subfolder contains all the configuration information (mostly in terms of environment variables) that are needed to run the various docker images that we are running (which depends on the configuration). 
 
 - The `calibrations` folder contains robot calibrations with a similar directory structure as is on the Duckiebot. 
 
### `launchers`

The `launchers` folder contains scripts that can be run by the agent. Specifically, the one that is indicated in the `config.yaml` file will be run by default when the agent container is run or when your exercise is submitted through the challenges infrastructure. 

Note: You can specify different launchers to run depending on whether you are testing/developing with the *exercises* infrastructure or submitting through the *challenges* infrastructure. 

### `notebooks`

The `notebooks` folder contains pedagogical notebooks that can be run. 

TODO: Vincenzo how to run the notebooks. 

The code in the notebooks can also be compiled and become accessible inside the code in the `exercise_ws` directory. 

TODO: Vincenzo more details on this. 

### `requirements.txt` 

The `requirements.txt` file contains any specific python requirements that you need for your submission. Note that these are requirements need over and above the *base image*. 

### `Dockerfile`

The `Dockerfile` contains the recipe for making your submission. In the normal case, this is relatively straightforward. We install the requirements, copy in the code and run a `launcher`. 


## The Exercises API

In the following we will describe the current commands that are supported within `dts exercises` and how they are used. 

### Building your code

You can start by building your code with:

    laptop $ dts exercises build

If you go inside the `exercises_ws` folder you will notice that there are more folders that weren't there before. These are build artifacts that persist from the building procedure because of mounting. 

Note: every time you run a `dts exercises` command you have to be inside an exercise folder or you will see an error. 
    

### Testing your code

With `dts exercises test` you can test your agent:

1. in the simulated environment,
2. on your robot but with the agent code running on your laptop,
3. with all of the code running on your robot.


#### Running in Simulation

You can run your current solution in the gym simulator with:
 
     laptop $ dts exercises test --sim
     
Then you can look at what's happening by looking through the "novnc" browser at http://localhost:8087 .

If you are running an exercise with a ROS-based baseline, you can use all of the existing ROS tools from this browser desktop. For example,  
open up the `rqt_image_view`, resize it, and choose `/agent/camera_node/image/compressed` in the dropdown. You should see the image from the robot in the simulator. 
 
You might want to launch a virtual joystick by opening a terminal and doing:
 
    laptop $ dt-launcher-joystick
     
If you are running the `duckietown_baseline`, by default the duckiebot is in joystick control mode, so you can freely drive it around. You can also set it to `LANE FOLLOWING` mode by pushing the `a` button when you have the virtual joystick active. If you do so you will see the robot move forward slowly and never turn. 

You might also explore the other outputs that you can look at in `rqt_image_view`. 

Also useful are some debugging outputs that are published and visualized in `RViz`. 
You can open `RViz` through the terminal in the `novnc` desktop by typing:


    laptop $ rviz
    
In the window that opens click "Add" the switch to the topic tab, then find the `segment_markers`, and you should see the projected segments appear. Do the same for the `pose_markers`. 

Another tool that may be useful is `rqt_plot` which also can be opened through the terminal in novnc. This opens a window where you can add "Topics" in the text box at the top left and then you will see the data get plotted live. 

All of this data can be viewed as data through the command line also. Take a look at all of the `rostopic` command line utilities. 

TODO: add pictures.

#### Testing Your agent on the Robot

If you are using a Linux laptop, you have two options, local (i.e., on your laptop) and remote (i.e., on the Duckiebot). If you are Mac user stick to the remote option. To run "locally"

    laptop $ dts exercises test --duckiebot_name ![ROBOT_NAME] --local

To run on the Duckiebot:

    laptop $ dts exercises test --duckiebot_name ![ROBOT_NAME]
    
In both cases you should still be able to look at things through novnc by pointing your browser to  http://localhost:8087 . If you are running on Linux, you can load up the virtual joystick and start lane following as above. 


#### Interactive Mode

You may find it annoying to completely shut down all of the running images and restart them to make a simple change to your code. To make things faster, you can use the `--interactive` flag with `dts exercises test`. 

In this case, when all of the containers other than the agent have started, you will be given a command line inside the agent container (overriding the comand specified in `config.yaml`. From here you can run your launcher from the command line manually.

Note: If you are running an exercise based on the `duckeitown_baseline` image, the first time you will have to start the "interface" part of the agent. To do this run 

    laptop container $ launchers/run_interface.sh. 
    
You will see some output of some ros nodes starting. At the end, if you push ENTER you will get your command line back. 
Then you can run the lane_following demo using your lane_controller by running 

    laptop container launchers/run_agent.sh 

You can do the normal thing of going to novnc and putting it into lane following mode or driving around with the joystick or whatever.

If you would like to change your code and re-run, just edit your code on your laptop, and then go to that terminal and do CTRL-C. You will see everything start to shut down. Then you can simply rerun the agent and it will have the new code that you just modified since it's mounted into the agent container. So just do `launchers/run_agent.sh` again and it will start up again.

Note: You will see an output from the anti-instagram node saying it's waiting for the first image. Don't worry, if you go to novnc and put the agent in lane following mode or drive with the joystick, it will start to receive images and that output will go away

Note: There is a timeout on the simulator, so if you do CTRL-C and then spend a while editing your code, it's likely that the simulator will have shut down. So either leave it running while you edit your code or just restart everything. 
You can get out of your terminal by typing 

    laptop container $ exit


  


