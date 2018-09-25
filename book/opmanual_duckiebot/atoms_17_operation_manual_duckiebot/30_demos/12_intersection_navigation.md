# Intersection Navigation  {#demo-inter-navigation status=draft}

This demo allows you to test the intersection navigation functionality. Your duckiebot will be able to cross intersections all by itself, just like a big duck.

<div class='requirements' markdown="1">

Requires: Duckiebot in configuration DB17-jwd.

Requires: Camera calibration completed.

Requires: Wheel calibration completed.

Requires: Duckietown Version 2.0.

</div>

## Video of expected results 

This video shows what you should get:

<div figure-id="fig:demo_video_navigators_op_man">
    <figcaption>Closed loop intersection navigation</figcaption>
    <dtvideo src="vimeo:258571104"/>
</div>


## Duckietown setup notes 

* Layout: Duckietown Version 2.0 with at least 1 intersection tile (4- or 3-way)
* Infrastructure: Traffic signs in the intersection tile according to D-2 3)
* Weather: Room or sunlight.

## Duckiebot setup notes 




## Pre-flight checklist 

Check: Is the battery is charged enough?

Check: Is your lens cover off?

Check: Is your bot up to date with latest software by using git pull?




## Demo instructions {#demo-intersection-navigation-run}

Follow these steps to run the current solution to the intersection navigation on your Duckiebot:

For the current working version you need to checkout the branch devel-intersection_navigation-jan15:

**Step 0**: On both the Duckiebot and the laptop switch to the correct branch:

```
git checkout devel-intersection_navigation-jan15
```

```
git pull
```

Finally source the environment and run :

```
catkin_make
```

**Step 1**: Place your Duckiebot at a four-way intersection just in front of the redline.

**Step 2**: The current version works with a gain of 0.6. To modify your gain to 0.6 run:

```
rosservice call /![robot name]/inverse_kinematics_node/set_gain -- 0.60
```

**Step 3**: On the Duckiebot, navigate to the `/DUCKIETOWN_ROOT/` directory, run the command:

```
source environment.sh
```

```
make demo-intersection-navigation
```

If you want to **visualize** what happens at the intersection when the template is matched, just follow these steps simultaneously with Step 3.

Navigate to the Duckietown folder on the laptop,

```
cd ~/duckietown
```

then source the environment,

```
source environment.sh
```

set the the ROS master to your vehicle,

```
source set_ros_master.sh ![robot name]
```

and finally launch

```
roslaunch intersection_navigation intersection_visualizer_node.launch robot_name:=![robot name]
```

## Troubleshooting

* My duckiebot does not initalize the navigation -> make sure to run the node with your robot name
* My duckiebot does not move -> Could happen if your battery is low, you did not remove the lid of the camera, no april tag is at the intersection or the duckiebot is not placed in front of an intersection


## Demo failure demonstration 

None existing yet.
