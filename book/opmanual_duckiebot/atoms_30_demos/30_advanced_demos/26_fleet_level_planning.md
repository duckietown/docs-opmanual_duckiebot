# Fleet planning Demo {#demo-fleet-planning status=beta}

The fleet planning demo demonstrates the implemented functionality for fleet level planning. I.e. an interactive taxi service.
It will provide you with a GUI running on a laptop that can be used to generate transportation requests. The Duckiebots will pick up the customer at its location and bring him/her to the final destination.

First, we describe what is needed, including:

* Robot hardware
* Number of Duckiebots
* Robot setup steps
* Duckietown hardware

<div class='requirements' markdown="1">

Requires: Duckiebot in configuration DB17-jwd. DB17-jwdl if coordination at intersections desired.

Requires: Camera calibration completed.

</div>

## Duckietown setup notes {#demo-fleet-planning-dt-setup}

You need at least one Duckiebot, and a Duckietown.

The Duckiebot needs to be at least a DB17-jwd, better a DB17-jwdl.

The Duckietown needs to have at least one april tag visible from every intersection to allow localization. Make sure to have a description of your map available. For more infos on this see the localization package. Or this [document](https://goo.gl/83TYBA). Also don't forget to transform the csv files to the xacro format using the corresponding script from the `Duckietown_description` package. Please note that the demo will NOT work if the Duckietown description is not correct.

If you use this demo, you will have to install the fleet communication dependencies. This might take up to 40 minutes. For more, see below.

## Duckiebot setup notes {#demo-fleet-planning-db-setup}

Duckiebots need to have all the dependencies installed. Use the fleet level communication setup developed by the fleet-communication team. To do this checkout the branch `devel-fleet-planning` and navigate into the folder `catkin_ws/src/30-localization-and-planning/fleet_messaging/dependencies`.

 Now identify the name of the wlan interface on by running: `ifconfig`. It is probably named `wlan0`.

 The next step is to run the install script as follows: `./install_fleet_messaging wlan_interface ip_address`.

 Where `wlan_interface` is the name of the wlan interface as found with `ifconfig` and `ip_address` is a randomly chosen IP address of the form `192.168.x.x/24`. Make sure that you run this install script with different IP addresses for each Duckiebot that takes part in the demo.

 As a last step, ensure that all the Duckiebots are connected to the same wifi network.


 ## Master Laptop setup notes {#demo-fleet-planning-master-laptop-setup}

 For the demo to run you need one laptop that is in the same network as all the Duckiebots and runs the central planning node. All Duckiebots report their location to that node. With that information it plans which Duckiebot picks up which customer. To receive this messages, the laptop also needs the fleet level communication setup. Perform the same steps as you just did on each Duckiebot on the laptop. (I.e. follow the instruction in the "Duckiebot setup notes" section. Running the installation script takes less time on the laptop than on the Duckiebot.

 ## Pre-flight checklist {#demo-fleet-planning-pre-flight}

 Before running the demo, check that:

 * Duckiebots have communication dependencies installed (described above).
 * Duckiebots and laptop are connected to the same network.
 * All Duckiebots can be ssh'ed from your laptop.
 * You got popcorn and refreshments for the taxi customers.

 ## Demo instructions {#demo-fleet-planning-run}

 ### Step 1:

 Pick a Duckiebot, log in via ssh.

 ### Step 2: checkout branch

 From Duckietown root folder:

     git checkout devel-fleet-planning

 ### Step 3: Environment

  Prepare environment:

     source environment.sh

     source set_vehicle_name.sh ![robot_name]

 ### Step 4: rebuild using catkin

     make build-catkin

 ### Step 5: Run the demo!

 Run this on the Duckiebot:

 CAUTION: For the Duckietown Demo Day on Jan 23 2018 you need to run the following command with the option `joystick_demo:=true`

     roslaunch fleet_planning master.launch messaging_iface:=![wlan_interface] messaging_config:=![config_file]

 Where `wlan_interface` is the interface you use to connect to the common network of all Duckiebots (probably `wlan0`) and `config_file` is the file needed to setup the communication. The files are provided as part of the fleet messaging package under the following path: `catkin_ws/src/30-localization-and-planning/fleet_messaging/config/config_Duckiebot_*.yaml`.

   Make sure to pick a different file for each Duckiebot. Make sure to provide an absolute path to the configuration file (i.e., `/home/![user]/Duckietown/catkin_ws/...`)

 Wait until all nodes have successfully been initialized. Then proceed with step 6.

 Option: Set `joystick_demo:=true` to switch off to take the autonomous control of the Duckiebot out of the loop. This way you can manually steer the Duckiebot through Duckietown and see how the fleet planning software works, with as litte dependency on other packages as possible. Pay attention to the terminal output of your Duckiebot to see which exit to take at an intersection. Give the Duckiebot time to localize at intersections.

 ### Step 6: Environment laptop

 On the laptop, checkout the same branch, rebuild catkin and in the Duckietown root folder:

     source environment.sh

 and

     source set_ros_master.sh

 Take care: *do not* pass an argument to the command `set_ros_master.sh`! We want a separate master to run on your laptop since the communication between Duckiebot and laptop is done using the fleet-communication software.

 ### Step 7: Taxi central:

 Run on the taxi central your laptop:

     roslaunch fleet_planning master_laptop.sh messaging_iface:=![wlan_interface] messaging_config:=![config_file]

 Where `wlan_interface` is the interface you use to connect to the common network with all Duckiebots (probably `wlan0`) and `config_file` is the file needed to setup the communication. The file is provided as part of the fleet messaging package under the following path `catkin_ws/src/30-localization-and-planning/fleet_messaging/config/config_laptop.yaml`.

 Make sure to provide an absolute path to the configuration file (i.e. `/home/![user]/Duckietown/catkin_ws/...`)

 ### Step 8: Start the GUI

     rqt --force-discover

 If you don't see nothing meaningful, start the fleet planning plugin via the menu `Plugins/Fleet Planning`.

 ### Step 9: Have fun!

 Place your Duckiebot at an intersection and it will localize and appear on the map in rqt. The taxi central will automatically assign a mission to the Duckiebot, random, to keep him moving and not blocking the streets. If you run the fleet planning in the standard mode, hit R1 on the joystick to go into auto pilot mode. The Duckiebot will now follow the instructions from the taxi central. If the `joystick_demo` is active, use the joystick to control the Duckiebot and follow the instructions from the Duckiebot's terminal output.

 Create a new customer request by clicking on the start node and then on the target node of your journey. Hit `Find Plan`. The taxi central will assign the customer to the closest Duckiebot and recalculate its path once it localizes again. The new path will be displayed on the map. You will see how the customer moves with its taxi once he was picked up.

 If a Duckiebot does not localize within a certain time window it will be removed from the map.

 ### Step 10: More Duckiebots!

 Once you get bored with only one Duckiebot on the map, or want to expand your business, add a another Duckiebot to your fleet by repeating steps 1-5. You may add a few more Duckiebots like this.

 ## Troubleshooting 

 The fleet planning demo depends on many other packages to work well. You may take the lane following and intersection control packages out of the loop by activating the joystick demo. More details at step 5.
