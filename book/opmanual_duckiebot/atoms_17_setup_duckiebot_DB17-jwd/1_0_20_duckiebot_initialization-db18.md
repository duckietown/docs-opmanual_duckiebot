# `DB18` Duckiebot Initialization {#setup-duckiebot-db18 status=draft}

Assigned: Breandan Considine, Liam Paull

<div class='requirements' markdown="1">

Requires: An SD card of dimensions at least 16 GB.

Requires: A computer with an internet connection, an SD card reader, and 16 GB of free space.

Results: A correctly configured Duckiebot SD card in configuration `DB18`. After assembling the Duckiebot, this will allow you to start it, connect to the internet, and get going.


</div>

## Install the Duckietown Shell

Install the most updated version of the Duckietown Shell, this is explained in [](+software_devel#dt-shell-intro).

If you already installed the Duckietown Shell, make sure it is updated by running

    $ dts update

## Initialize the SD card

Plug the SD card in the computer using the card reader. Then initalize it by running the commands

    $ dts init_sd_card

Then the terminal will ask you some configuration parameters, as the username you wish to use in your Duckiebot as well as various passwords. Follow the instructions to complete the setup.
If you whish to leave the default parameters press enter without writing anything.
It is strongly encouraged not to leave the default password but rather to choose a personal one. In this configuration step you will as well be asked to insert the credentials for the Wi-Fi you want to use. By default your Duckiebot will connect to a network named `duckietown` with password `quackquack`, but you can change that to match the network to which you are connected with yout computer.
