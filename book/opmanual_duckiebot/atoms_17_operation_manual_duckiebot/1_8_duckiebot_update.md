# Keeping your Duckiebot up to date {#duckiebot-autoupdate status=ready}

<div class='requirements' markdown="1">

Requires: A Duckiebot that has been initialized

Requires: A computer with a **Ubuntu OS**,

Requires: an internet connection to the Duckiebot, configured as in [](#duckiebot-network).

Requires: Duckietown Shell, Docker, etc., as configured in [](#laptop-setup).

Requires: Duckietown Token set up as in [](#dt-account).

Results: An up to date Duckiebot!

</div>

## Understand what is the difference between OTA update and Release Update {#dt-what-update status=ready}

The update method described in this page will allow you to receive an **On The Air** (OTA) update within the distribution you so choose (e.g., `daffy`), and it can improve your Duckiebot performance. Note that this is different from using the `init_sd_card` tool: Using the `init_sd_card` tool will only provide the latest release version, not the latest Duckiebot software version.

## Update Duckiebot container using `dts` command {#dt-autoupdate-dts status=beta}

If your Duckiebot has not been used for a while and a new image has been released, you don't necessarily need to re-flash the Duckiebot image as described in [the initialization procedure](#setup-duckiebot). Instead, you can use `dts duckiebot update` command to update your Duckiebot.

    laptop $ dts duckiebot update ![DUCKIEBOT_NAME]

You will see a prompt similar to this:

<div figure-id="fig:dt-autoupdate.png" figure-caption="Auto Update Duckiebot Container">
     <img src="dt-autoupdate.png" style='width: 25em'/>
</div>

Type in `y` for yes to continue updating. If you would like to abort, you can use <kbd>Ctrl</kbd>-<kbd>C</kbd> to stop the update.

Note: This process is expected to take a while to complete.

## Update Duckiebot container using Dashboard {#dt-autoupdate-dashboard status=beta}

To use the Dashboard for updating the Duckiebot container, first navigate to the Dashboard software page. If you are not sure how to do that, refer to [this](#dashboard-tutorial-software) page. If you have containers that need to be updated, you will see the update button next to the container:

<div figure-id="fig:dashboard-update1" figure-caption="">
  <img src="dashboard-update1.png" style='width: 35em'/>
</div>

The update will proceed upon click. If update is successful, you will see the image below:

<div figure-id="fig:dashboard-update2" figure-caption="">
  <img src="dashboard-update2.png" style='width: 35em'/>
</div>
