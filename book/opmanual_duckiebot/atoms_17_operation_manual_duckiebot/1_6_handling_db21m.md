# Handling - Duckiebot `DB21M` {#handling-duckiebot-db21m status=ready}

<div class='requirements' markdown="1">

Requires: An assembled `DB21` or `DB21M`. Find the assembly instructions [here](#assembling-duckiebot-db21m).

Requires: An initialized `DB21` or `DB21M` with **image version at least 1.2.2**. Find the initialization instructions [here](#setup-duckiebot). [Check your current firmware version](#duckiebot-dashboard-use) before proceeding.

Result: Knowledge on standard protocols to turn on, turn off, charge, and update the Duckiebattery software version on a `DB21` or `DB21M`.

</div>

Note: the above box contains important information on the requirements. Make sure to read and follow them before proceeding.

## Duckiebot `DB21M` handling tutorial video {#tutorial-handling-db21m-video}

<div figure-id="fig:howto-handle-db21m" figure-caption="Duckiebattery power on, shutdown and charging protocols.">
    <dtvideo src="vimeo:527038785"/>
</div>

## How to charge a `DB21M` {#howto-db21m-charge status=ready}

To charge your Duckiebot, follow these steps:

- Plug in the charging cable to the free microUSB port on the `HUT`.

Note: to minimize mechanical stress on the `HUT` we recommend plugging in the charging cable once, and leaving the USB port end free to plug and unplug from charging instead. You can arrange the cable under the `DB21M` top plate during operations for cable management.

- Plug in the charger to a 5V 2A source.

Note: the battery can draw up to 2A. Feeding a higher amperage will not be a problem, but wrong voltage will send the battery in [protection mode](#db-opmanual-preliminaries-battery-protection).

- If the Duckiebot is turned on when charging, a battery charge indicator will appear on the top right of the screen. If the Duckiebot is turned off, the LEDs will turns on. In both cases, a small LED on the `HUT` near the charger port will turn green, indicating incoming power.

## How to power off a `DB21M` {#howto-db21m-shutdown status=ready}

Warning: The proper shutdown protocol for a `DB21M` requires having the Duckiebattery software version 2.0.0 or later. To check the version of your battery, follow the instruction to "Verify current battery version" on [How to update a Duckiebattery](#howto-db21m-battery-update).  

Make sure the Duckiebot has completed the booting process. You can verify this by checking the "Status" after running `dts fleet discover` on your laptop: a green `Ready` message will indicate that the Duckiebot has completed the booting process.

Note: There are three methods to power off a DB21M (recommended method: "With the top button"):

1. With the **top** button:
    1. Press the **top** button (not the battery button) for 5 seconds and release.
    1. What to expect:
        1. The user should see the top button blinks for 3 seconds
        1. the Duckiebot front and back LEDs should be turned off
        1. Then in about *10* seconds, the Jetson and fan should stop.
    1. Troubleshooting: If the display just switched to the next page and the top button did not blink, try again and push harder on the top button during the 5 seconds.
1. With `duckietown-shell`:
    1. `dts duckiebot shutdown ![hostname]`
    2. What to expect:
        1. The Jetson and fan should stop in about *10* seconds.
        1. If the charging cable is not attached, the front and back LEDs should also be turned off.
1. With the Duckiebot dashboard:
    1. Open a browser
    1. Navigate to `http://![hostname].local`
    1. In the Top-Right corner, click on the `Power` options, and choose "`Shutdown`". Then confirm the action.
    1. What to expect: the same as the "With `duckietown-shell`" method.

Warning: The following "hard" power shutdown should be only be used if the three methods above failed to shutdown the Duckiebot, because it might lead to software and hardware issues. 

As a last resort, one could still perform a "hard" power shutdown of the `DB21M`:
- `ssh duckie@![hostname].local sudo poweroff`
- unplugging the micro USB cable from the port marked as `5Vraspi` on the `HUT`.   


## How to power on a `DB21M` {#howto-db21m-poweron status=ready}

To power on a Founder Edition Duckiebot, press the button on the battery _once_.

The Duckiebot LEDs, as well as the Jetson Nano board booting LED will turn on.

After a few seconds, the WiFi dongle will start blinking. The Duckiebot LEDs will then turn to a steady white color, followed by the button and screen on the top plate powering on, as shown in the [tutorial video](#fig:howto-handle-db21m).   

## How to update a Duckiebattery {#howto-db21m-battery-update status=ready}

To update the software running on the micro-controller in the Duckiebattery, or just checking the current version of it, follow this procedure.

When reporting issues on Stack Overflow, please include the step number, e.g. _Step 4.i.b_, the actions performed, and a description of the unexpected/unknown outcome.

***Important:***

1. Before the battery upgrade, please make sure the battery has at least 15% of charge.
2. Run all following commands on the desktop/laptop

Make sure the Duckiebot is powered on and connected to the network. You can verify the latter by launching, e.g., `dts fleet discover` and finding that your Duckiebot is on the list. 

All following `![hostname]` refers to the name of the Duckiebot to which the battery is plugged in.


1. Please update the `duckietown-shell` utility:
    1. `pip3 install --user --upgrade --no-cache-dir duckietown-shell`
    2. `dts update`
    3. `dts desktop update`
2. Update the Duckiebot:
    1. ```dts duckiebot update ![hostname]```
3. Reboot the Duckiebot:
    1. `ssh duckie@![hostname].local sudo reboot`
    2. Wait until the Duckiebot reboots and the display shows information (especially about the battery).
    3. You could verify the battery related software is up and running by checking  whether the display reacts correctly to charging states when a charging cable is plugged in and unplugged. 
4. Upgrade the battery firmware:
    1. `dts duckiebot battery upgrade ![hostname]`
        1. Note: When prompted to "double-click" on the battery button, make sure to _quickly_ click twice the _battery_ button.
        2. Note: Do not worry if you are unsure if you actually pressed the button twice or not, as the battery upgrade process will verify this.
        3. Follow the instructions in the terminal.
    2. If the command finished with the error: ```SAM-BA operation failed INFO:UpgradeHelper:An error occurred while flashing the battery. ERROR:dts:The battery reported the status 'GENERIC_ERROR'```, please try flashing again with: `dts duckiebot battery upgrade --force ![hostname]`
    3. If the command finished with any other error: **single** press the battery button, and start from _step 3_ again one more time. If there are still errors, please report on StackOverflow.
5. Prepare for post-upgrade checks
    1. If the battery indicates the charging states correctly, and shows the percentage number as normal, proceed to _step 6_
    2. If the display shows "`NoBT`" (No battery detected), then **single** press the battery button, and run:
        1. `ssh duckie@![hostname].local sudo reboot`
        2. Wait for the reboot (as described in _step 3_)
        3. Then proceed to _step 6_
6. Verify current battery version:
    1. Method 1:
        1. `dts duckiebot battery check_firmware ![hostname]`
        2. Verify the battery version should be `"2.0.2"`
    2. Method 2:
        1. Open a browser window
        2. Navigate to `http://![hostname].local/health/battery/info`
        3. Verify the battery version should be `"2.0.2"`


## How to update a `HUT` {#howto-hut-update status=ready}

Instructions on how to flash a Duckietown `HUT` board can be found [here](#reflash-microcontroller).

Note: (re)flashing a `HUT` is typically not needed. A notable exception is for `HUT` version 3.15 which comes with `DB21`s. The `HUT` version can be read on the board itself. 