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

Since we are not going to use Google Sign-In, you can click on **Skip**.
This will let you skip the first two steps and move straight to **Step 3**.
Do not worry about creating an administrator account (Step 2) for now,
the Duckietown package for \\compose\\ will create one for us as soon as we
authenticate for the first time using our personal token.

### Step 3

At this point, the **Step 3** tab should be open, as shown in the image below.

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

Since we skipped the first two steps of the **First Setup** of \\compose\\
([](#monitor-first-boot)), we cannot login using a Google account.
As discussed above, the Duckietown package provides its own login system,
which is based on the Duckietown personal token (Duckietoken).
If you have not retrieved yours, it is now time to do so by visiting
the page:

> [`https://www.duckietown.org/site/your-token`](https://www.duckietown.org/site/your-token)

Note: Since your dashboard does not have an administrator account yet,
the first user to login will be automatically assigned the role of
administrator. If you have multiple tokens, make sure to keep note of
which one you used for the first login.

Once you have your personal token, you can go ahead and click on
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

As you might have noticed, the left hand side bar now shows many more pages. Some pages are
accessible by all users, others only by administrators (e.g., Settings,
Package Store, Debug).

Also, you might have noticed that some Duckietown-specific pages are already
there (e.g., Portainer, Mission Control, Duckietown). This is because we used
a Docker image of \\compose\\ that comes with some Duckietown packages
pre-installed.

Take your time to visit all the pages and get comfortable with the platform.
We will discuss the functionalities offered by each page in the next sections.

## What is in the dashboard? {#dashboard-overview status=ready}

To see all the available components within the dashboard, you will need to first login to the dashboard. Inside the dashboard, you will see a navigation panel on your left hand side. There are 7 subpages of dashboard. They are:

    PAGE NAME           DESCRIPTION
    Portainer           A nice GUI tool for seeing all containers running on a Duckiebot
    Robot               A summary page for the robot status
    Users               Advanced Feature: Allow multiple student using one Duckiebot
    Profile             Information for your duckietown account
    Package Store       A package store contain all available packages for your Duckiebot
    Settings            Advanced Feature: Change configuration of your Duckiebot dashboard manually
    Restful API         Advanced Feature: A description of restful API.

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

### Software {#dashboard-tutorial-software status=ready}

This is the software page. You can use this page to monitor your container status. You can select to update the containers you desired. See also [here](#dt-autoupdate-dashboard) for more information about keeping your Duckiebot up to date.

<div figure-id="fig:dashboard-software" figure-caption="">
  <img src="dashboard-software.png" style='width: 35em'/>
</div>

### Files {#dashboard-tutorial-files status=ready}

This is the files page. You can access your calibration result directly through this tab. You calibration are stored under `/config/calibration`. If you do not want to recalibrate everytime you re-flahsed your SD card, it is recommended to save these calibration results.

Additionally, you can access configuration files Duckiebot uses in this tab.

<div figure-id="fig:dashboard-files" figure-caption="">
  <img src="dashboard-files.png" style='width: 35em'/>
</div>

## Dashboard Package Store {#dashboard-update status=beta}

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
