# Operation - Use the Dashboard {#duckiebot-dashboard-use status=ready}

<div class='requirements' markdown='1'>

Requires: Laptop configured, according to [](#laptop-setup).

Requires: You have configured the Duckiebot as documented in [](#setup-duckiebot).

Requires: You have completed the Dashboard setup as documented in [](#duckiebot-dashboard-setup).

</div>

This section shows how to use the Duckietown Dashboard on the Duckiebot.

## What is in the Dashboard? {#dashboard-overview status=ready}

The following video provides a brief tour of the most important features
of the Duckietown Dashboard on your Duckiebot.

<div figure-id="fig:howto-dashboard-use" figure-caption="Dashboard operation tutorial.">
<dtvideo src="vimeo:527022343"/>
</div>

To see all the available components within the dashboard, you will need to first login to the dashboard. Inside the dashboard, you will see a navigation panel on your left hand side. There are 7 subpages of dashboard. They are:

    PAGE NAME           DESCRIPTION
    Portainer           A nice GUI tool for seeing all containers running on a Duckiebot
    File Manager        A file manager for managing the files on the robot
    Robot               A summary page for the robot status
    Profile             Information for your duckietown account
    Package Store       A package store contain all available packages for your Duckiebot
    Users               Advanced Feature: Allow multiple student using one Duckiebot
    Settings            Advanced Feature: Change configuration of your Duckiebot dashboard manually
    Restful API         Advanced Feature: Documentation to the RestAPI exposed by the Dashboard.

## Portainer {#dashboard-portainer status=ready}

Portainer is a provided tool for managing all the docker containers that are running on the Duckiebot. Using portainer tools, you can quickly see the status of the containers on your Duckiebot.

<div figure-id="fig:dashboard-portainer" figure-caption="">
  <img src="dashboard-portainer.png" style='width: 35em'/>
</div>

You can select `containers` to see all the containers on the Duckiebot.

For more information about portainer, you can find them in [this](#sub:dashboard-portainer) page.

## Robot Page {#dashboard-robot status=ready}

In this page you will find several tabs that help you see and understand the Duckiebot status. The default tab is `Info`.

### Info {#dashboard-robot-overview status=ready}

In this tab, you can find information for your robot, including your robot name, type, configuration, and critical information such as CPU usage, temperature, and other crucial robot vitals.

<div figure-id="fig:dashboard_info" figure-caption="">
  <img src="dashboard_info.png" style='width: 35em'/>
</div>

Note: from this page you can read the Duckiebot's firmware version, i.e., the version of the base image used during [initialization](#setup-duckiebot).

### Mission Control {#dashboard-mission-control status=ready}

This is the Mission Control tab.

<div figure-id="fig:dashboard_mission_control_auto" figure-caption="">
  <img src="dashboard_mission_control_auto.png" style='width: 35em'/>
</div>

In this tab you can see what the Duckiebot sees and you can see lateral and angular speed of your robot, and a plot of left and right motor speed.
This is the tab that lets you monitor and control your Duckiebot. The top of the page should be similar to the following image,

Note: If you do not see the camera view, make sure you are accessing the dashboard using `https://ROBOT_HOSTNAME.local/` instead of directly accessing the dashboard using robot IP address.

**Did you know?**
The page contains 4 blocks by default.
Feel free to drag them around and rearrange them as you please.
You can also use the menu button of each block to resize them.

### Health {#dashboard-robot-health status=ready}

This is the Health Page. It will show you a plot of the robot's health status such as temperature, frequency, and CPU usage. It is a good debug too to watch your code's resource usage.

<div figure-id="fig:dashboard-health" figure-caption="">
  <img src="dashboard-health.png" style='width: 35em'/>
</div>

### Architecture {#dashboard-robot-architecture status=ready}

This is the Architecture Page. It will allow you to visualize all the published ROS topics and see their details. It is a useful tool to see what is running and what is not. You can also use this tool in replacement of `rqt-graph`. For more instructions on rqt-graph, you can see it [here](#rqt-graph-no-vnc)

<div figure-id="fig:dashboard-architecture" figure-caption="">
  <img src="dashboard-architecture.png" style='width: 35em'/>
</div>


## Drive your Duckiebot via mission control {#drive-dashboard status=draft}

You can remotely drive your Duckiebot through mission control page.

The first thing to check to make sure that everything we have done so far
is correct, is the status of the **Bridge**, in the top-right corner of the page.
The label should show the status "**Bridge: Connected**" (as shown in the image above).
If the indicator reads "**Bridge: Closed**", it means that something went wrong
while launching the ROS websocket node above. In that case, start again from
the beginning of this section.

Note: Don't worry if one of the blocks is called "Camera" but you
don't see an image. We will get to that later.

Toggle the **Take over** switch
in the top-right corner of the page to gain control of your robot.
You will see that the background of the page will highlight and the
central plot will start moving.

You can now use the arrows on your keyboard to drive your Duckiebot.
