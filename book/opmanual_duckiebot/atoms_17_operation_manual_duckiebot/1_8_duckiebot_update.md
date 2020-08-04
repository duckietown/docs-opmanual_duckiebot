# Keeping your duckiebot up to date {#duckiebot-autoupdate status=ready}

<div class='requirements' markdown="1">

Requires: A duckiebot that has been initialized

Requires: A computer with a **Ubuntu OS**, an internet connection to duckiebot

Requires: Duckietown Shell, Docker, etc, as configured in [](#laptop-setup).

Requires: Duckietown Token set up as in [](#dt-account).

Results: An up to date duckiebot!

</div>

## Update duckiebot container using dts command {#dt-autoupdate-dts status=ready}

If your duckiebot has not been used for a while, and there has been new image released for the duckiebot, you don't necessarily need to reflash the image of the duckiebot, instead you can use `dts duckiebot update` command to update your duckiebot.

    laptop $ dts duckiebot update ![DUCKIEBOT_HOSTNAME]

You will see a prompt similar to this:

<div figure-id="fig:dt-autoupdate.png" figure-caption="Auto Update Duckiebot Container">
     <img src="dt-autoupdate.png" style='width: 25em'/>
</div>

Type in `y` for yes to continue updating. If you would like to abort, you can use CTRL-C to stop the update.

Note: This process will take a while to complete, it is recommended to take a coffee break while executing the command.

## Update duckiebot container manually {#dt-manualupdate status=beta}

Instead of automagically updating the container using `dts`, you can also update the container manually using docker commands.

