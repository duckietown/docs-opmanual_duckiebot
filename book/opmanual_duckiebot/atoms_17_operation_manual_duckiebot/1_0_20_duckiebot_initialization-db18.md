# `DB18` Duckiebot Initialization {#setup-duckiebot-db18 status=draft}

Assigned: Breandan Considine, Liam Paull

<div class='requirements' markdown="1">

Requires: An SD card of dimensions at least 16 GB.

Requires: A computer with a **Ubuntu OS**, an internet connection, an SD card reader, and 16 GB of free space.

Results: A correctly configured Duckiebot SD card in configuration `DB18`. After assembling the Duckiebot, this will allow you to start it, connect to the internet, and get going.

</div>

## Burn the SD card {#burn-sd-card}

First you must install the [Duckietown Shell Environment](https://github.com/duckietown/duckietown-shell). To do so follow the instruction in the [Duckietown Shell README](https://github.com/duckietown/duckietown-shell/blob/master/README.md).

Next execute the command:

​    laptop $ dts init_sd_card

and follow the instructions. 

- When you are asked if you should overwrite your SSH identity file select <kbd>y</kbd>

- Enter a username (default is `duckie` - we will call this `![username])

- Enter a password to login to robot via ssh (default is `quackquack`)

- Enter a hostname for your robot = the "name" of your robot (default is `duckiebot` - we will call this `![hostname])

- Enter a  WiFi SSID for your robot to connect to by default (default is `duckietown` - we will call this `![wifi-ssid])

- Enter the Wifi password (default is `quackquack`)

- Enter the Duckiebot SSID of the network that the robot will broadcast (default is `![hostname]` - we will call this `![duckiebot-ssid])

- Enter the password for your Duckiebot Wifi (default is `quackquack`)

- You will then have to enter your laptop's `sudo` password to run Etcher

- Select the drive at `/dev/mmcblk0` by pressing <kbd>Enter</kbd>

- When asked "Are you sure?" select <kbd>y</kbd>


When the SD card is completely written, you should arrive at `Press any key to continue`. Do so and the script will exit. You can then remove the SD card from your laptop. 



## Booting the Duckiebot {#duckiebot-boot}

Now insert the SD card into the Raspberry pi and push the button on the battery to power things up. You should immediately see the **green** light next to where the SD card was inserted start to blink with activity. If not stop, there is a problem with the SD card (or possible the Pi but this is unlikely).

Warning: Allow the robot time to boot. On first boot it may take up to 5 mins or more since some things are being configured. Do not power the robot off (by holding the battery button) while this is in process. 

You know that your Pi has successfully booted when you can see it broadcasting a network with an SSID of `![hostname]-XXXX` where `XXXX` are some random numbers and letters at the end of SSID to prevent name conflicts.

If you connect to the newtork `![hostname]-XXXX` or to the network that the Duckiebot connects to by default ![wifi-ssid], then you should be able to ping your robot with:

​    laptop $ ping ![hostname].local

You should see output similar to the following:

​    PING duckiebot-not-configured.local (![X.X.X.X]): 56 data bytes
​    64 bytes from ![X.X.X.X]: icmp_seq=0 ttl=64 time=2.164 ms
​    64 bytes from ![X.X.X.X]: icmp_seq=1 ttl=64 time=2.303 ms
​    ![...]

