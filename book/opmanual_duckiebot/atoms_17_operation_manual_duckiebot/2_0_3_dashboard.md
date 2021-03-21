# Setup - Dashboard {#duckiebot-dashboard-setup status=ready}

This section shows how to install the Duckietown dashboard on the Duckiebot using Docker.

<div class='requirements' markdown='1'>

Requires: Laptop configured, according to [](#laptop-setup).

Requires: You have configured the Duckiebot as documented in [](#setup-duckiebot).

Requires: You have configured Docker communication as documented in [](#docker-setup).

Results: You have access to a web-based Dashboard for your Duckiebot.

</div>

## The \compose\ platform {#compose-platform}

\\compose\\ is a CMS (Content Management System) platform that provides functionalities
for fast-developing web applications. Custom applications are developed as external
packages that can be installed using the built-in Package Store.

The Duckiebot Dashboard is a package that you can install on your instance
of \\compose\\ running on your Duckiebot.
To make is easier for you to get started, we provide a Docker image with \\compose\\
and all the packages you need. Follow the instructions in the next step to get started.

Visit the
[official documentation page](http://compose.afdaniele.com/docs/latest/index)
for further information about \\compose\\.

## Setting up dashboard {#init-dashboard}

You can find your duckietown dashboard at:

    http://![YOUR_DUCKIEBOT_NAME].local/

If the above address does not work, remove the `.local` part and just do `http://![YOUR_DUCKIEBOT_NAME]/`

Note: if `.local` does not work, that means your router's default domain name is set to something else. It will be helpful if you figure out what that is. And keep in mind that any instruction later that includes `.local` should be just ignored.

### Steps 1, 2

By default, \\compose\\ uses Google Sign-In to authenticate the users.
In Duckietown, we use authentication based on personal tokens. You should be able to
retrieve yours by visiting the page:

> [`https://www.duckietown.org/site/your-token`](https://www.duckietown.org/site/your-token)

You should notice that the first two steps already appear to be completed. 
Do not worry about creating an administrator account (Step 2) for now,
a new aministrator account will be automatically created the first time we login 
using a Duckietown token.

### Step 3

**Step 3** is about configuring the dashboard.

<div figure-id="fig:compose_first_setup_step3" figure-caption="">
  <img src="compose_first_setup_step3.png" style='width: 34em'/>
</div>

You can complete this step as you please.
Feel free to update all the fields, and remember, you can always update your
choices through the page **Settings** after you authenticate
using your personal token.

When you are happy with your choices, click on **Next**.

### Step 4

The **Step 4: Complete** tab should now be open, as shown below.

<div figure-id="fig:compose_first_setup_step5" figure-caption="">
  <img src="compose_first_setup_step5.png" style='width: 34em'/>
</div>

You can go ahead and press **Finish**.

## First Login

If everything went as planned, the dashboard is now configured and ready to go!

You should be able to see the login page, as shown below.

<div figure-id="fig:dashboard_login_page" figure-caption="">
  <img src="dashboard_login_page.png" style='width: 34em'/>
</div>

Note: Since your dashboard does not have an administrator account yet,
the first user to login will be automatically assigned the role of
administrator. If you have multiple tokens, make sure to keep note of
which one you used for the first login.

If you have not retrieved your personal Duckietown Token as described in [](#dt-account) yet, 
it is now time to do so.
Once you have your personal Duckietown token, go ahead and click on
the button **Sign in with Duckietown**.
You should now see a dialog like the one shown below,

<div figure-id="fig:dashboard_login_with_duckietown_modal" figure-caption="">
  <img src="dashboard_login_with_duckietown_modal.png" style='width: 35em'/>
</div>

Copy/paste or type in your personal token and press Login.
Wait for the token to be validated, and if your token is correct, you will
be redirected to your profile page, similar to the one shown below.

<div figure-id="fig:dashboard_profile_page" figure-caption="">
  <img src="dashboard_profile_page_full.png" style='width: 35em'/>
</div>

As you might have noticed, the side bar to the left now shows many more pages. Some pages are
accessible by all users (e.g., Robot), others only by administrators (e.g., Settings,
Package Store).

Take your time to visit all the pages and get comfortable with the platform.
We will discuss the functionalities offered by each page in the next sections.

## What is in the dashboard? {#dashboard-overview status=ready}

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

For more information about portainer, you can find them in [this](#docker-setup-portainer-interface) page.

## Robot Page {#dashboard-robot status=ready}

In this page you will find several subpage that helps you see and understand the Duckiebot status. The default page is `Info`.

### Info {#dashboard-robot-overview status=ready}

In this page, you can find information for your robot, including your robot name, type, configuration, and critical information such as CPU usage, temperature, and other crucial robot vitals.

<div figure-id="fig:dashboard_info" figure-caption="">
  <img src="dashboard_info.png" style='width: 35em'/>
</div>

### Mission Control {#dashboard-mission-control status=ready}

This is the Mission Control page.

<div figure-id="fig:dashboard_mission_control_auto" figure-caption="">
  <img src="dashboard_mission_control_auto.png" style='width: 35em'/>
</div>

In this page you can see what the Duckiebot sees and you can see lateral and angular speed of your robot, and a plot of left and right motor speed.It is the page that lets you monitor and control your Duckiebot. The top of the page should be similar to the following image,

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

TODO: (afdaniele) add screenshot of the File Manager page


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
