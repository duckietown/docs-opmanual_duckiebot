# Handling - Duckiebot `DB21M` {#handling-duckiebot-db21m status=ready}

<div class='requirements' markdown="1">

Requires: An assembled `DB21M`. Find the assembly instructions [here](#assembling-duckiebot-db21m).

Requires: An initialized `DB21M` with image version at least 1.2.2. Find the assembly instructions [here](#assembling-duckiebot-db21m).

Result: Knowledge on standard protocols to turn on, turn off, charge, and update the Duckiebattery software version on a `DB21M`.

</div>

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

The proper shutdown protocol for a `DB21M` requires having Duckiebattery software version 2.0. To check the version of your battery, follow the instruction on [how to update a Duckiebattery](#howto-db21m-battery-update).  

Make sure the Duckiebot has completed the booting process. You can verify this by checking the "Status" after running `dts fleet discover` on your laptop: a green `Ready` message will indicate the Duckiebot has completed the booting process.

You have three ways to power off your Duckiebot:

- Press the button on the top plate of the Duckiebot, near the screen, and keep it pressed for roughly 5 seconds before releasing it. The shutdown sequence will initiate with the blinking of the power button and a "Shutting down" message appearing on the screen.
- From the terminal run:

    laptop $ dts duckiebot shutdown ![DUCKIEBOT_NAME]
    
- From the dashboard, running at `http://![DUCKIEBOT_NAME].local`, select the tab `ROBOT` from the sidebar on the left, click the button `Power` and select `shutdown` on the top right corner.


The shutdown sequence will first turn off the LEDs, then the screen, then computational unit, and finally the fan. Note that this is a "soft" shutdown procedure, which correctly terminates the processes running on the Jetson Nano board.  


## How to power on a `DB21M` {#howto-db21m-poweron status=ready}

To power on a Founder Edition Duckiebot, press the button on the battery _once_.

The Duckiebot LEDs, as well as the Jetson Nano board booting LED will turn on.

After a few seconds, the WiFi dongle will start blinking. The Duckiebot LEDs will then turn to a steady white color, followed by the button and screen on the top plate powering on, as shown in the [tutorial video](#fig:howto-handle-db21m).   

## How to update a Duckiebattery {#howto-db21m-battery-update status=ready}

To update the software running on the micro-controller in the Duckiebattery, or just checking the current version of it, follow the following procedure.

- Watch this tutorial video:


<div figure-id="fig:howto-battery-update-db21m" figure-caption="Duckiebattery software upgrade tutorial.">
    <dtvideo src="vimeo:526718185"/>
</div>


- Make sure the Duckiebot is powered on and connected to the network. You can verify the latter by launching, e.g., `dts fleet discover` and finding that your Duckiebot is on the list.


- Open a terminal on the laptop and run `dts duckiebot battery upgrade ![hostname]`, where `![hostname]` is the name of the Duckiebot to which the battery is plugged in.


If a Duckiebattery is detected, the prompt will show the currently installed version of the software as well as the latest one available.

Note: if you wish to just check the software version of the code, press `n` now and abort the process.


- To initiate the battery software upgrade procedure, type `y` and press <kbd>Enter</kbd>.  


- The terminal will then prompt to press the button on the Duckiebattery _twice_. This operation will trigger a special `boot` mode for the battery, necessary to update its software. Do so, then go back to the terminal and press <kbd>Enter</kbd>.

Note: the Duckiebot will not give any visible sign of the battery having entered boot mode. Do not worry if you are unsure if you actually pressed the button twice or not, as the battery upgrade process will verify this.

- Wait until the procedure is complete.

Congratulations, your battery software is up to date!


<!--

The program is now talking to the battery to figure out whether an update is necessary.

As we can see, in this case the battery is running the software version 1.0 while the version 2.0 is available. We will be asked if we want to update now, and we confirm by typing "y" and pressing ENTER.

The program is now ready to transfer the new software to the battery, but we have to tell the battery to get ready for an incoming update.

 We can do so by putting the battery into the so-called "Boot Mode" by pressing the button on the battery twice in a row.

When we are done, we press ENTER on the terminal.

 Do not worry if you are not sure the double press was done properly, the program will tell us if we need to try again.


The message "Updating battery" is telling us that the battery is now receiving the new code, let's wait.

Well done, the battery is now updated and ready to go back to work.


-->
