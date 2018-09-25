# Duckiebot Initialization {#setup-duckiebot status=ready}

Assigned: Breandan Considine, Liam Paull

This page is for the `DB18` configuration used in classes in 2018. For last year's instructions see [here](docs.duckietown.org/17/). 

<div class='requirements' markdown="1">

Requires: An SD card of size at least 16 GB.

Requires: A computer with a **Ubuntu OS** (for flashing the SD card), an internet connection, an SD card reader, and 16 GB of free space.

Results: A correctly configured Duckiebot SD card in configuration `DB18`. After assembling the Duckiebot, this will allow you to start it, connect to the internet, and get going.

</div>

## Install the Duckietown Shell

Install the most updated version of the Duckietown Shell, this is explained in [the duckietown-shell README](https://github.com/duckietown/duckietown-shell/blob/master/README.md).

If you already installed the Duckietown Shell, make sure it is updated by running

```
$ dts update
```

## 

## Burn the SD card {#burn-sd-card}

warning: this currently only works on Ubuntu OS

Plug the SD card in the computer using the card reader. Then initalize it by running the command:

```
laptop $ dts init_sd_card
```

and follow the instructions:

- When you are asked if you should overwrite your SSH identity file select <kbd>y</kbd>

- Enter a username (default is `duckie` - we will call this `![username]`)

- Enter a password to login to robot via ssh (default is `quackquack`)

- Enter a hostname for your robot = the "name" of your robot (default is `duckiebot` - we will call this `![hostname]`)

- Enter a  WiFi SSID for your robot to connect to by default (default is `duckietown` - we will call this `![wifi-ssid]`)

- Enter the Wifi password (default is `quackquack`)

- Enter the Duckiebot SSID of the network that the robot will broadcast (default is `![hostname]` - we will call this `![duckiebot-ssid]`)

- Enter the password for your Duckiebot Wifi (default is `quackquack`)

- You will then have to enter your laptop's `sudo` password to run Etcher

- Select the drive at `/dev/mmcblk0` by pressing <kbd>Enter</kbd>

- When asked "Are you sure?" select <kbd>y</kbd>


When the SD card is completely written, you should arrive at `Press any key to continue`. Do so and the script will exit. You can then remove the SD card from your laptop. 



## Booting the Duckiebot {#duckiebot-boot}

Now insert the SD card into the Raspberry pi and push the button on the battery to power things up. You should immediately see the **green** light next to where the SD card was inserted start to blink with activity. If not stop, there is a problem with the SD card (or possible the Pi but this is unlikely).

Warning: Allow the robot time to boot. On first boot it may take up to 5 mins or more since some things are being configured. Do not power the robot off (by holding the battery button) while this is in process. 



You know that your Pi has successfully booted when you can see it broadcasting a network with an SSID of `![hostname]-abcd` where `abcd` are some random numbers and letters at the end of SSID to prevent name conflicts.



If you connect to the newtork `![hostname]-abcd` or to the network that the Duckiebot connects to by default ![wifi-ssid], then you should be able to ping your robot with:

```
laptop $ ping ![hostname].local
```

You should see output similar to the following:​    

```
PING duckiebot-not-configured.local (![X.X.X.X]): 56 data bytes
64 bytes from ![X.X.X.X]: icmp_seq=0 ttl=64 time=2.164 ms
64 bytes from ![X.X.X.X]: icmp_seq=1 ttl=64 time=2.303 ms
![...]
```



## SSH to the Duckiebot

Next, try to log in using SSH

```
laptop $ ssh ![username]@![hostname].local
```

and enter your password.

It is recommended that you immediately give ownership of your home folder to your user. You can do this by running the following on your Duckiebot

```
duckiebot $ sudo chown -R ![username]:![username] .
```

You should place your SSH key onto the robot so that you don't need to enter it in the future. This can be done with:

```
laptop $ ssh-copy-id -i ~/.ssh/mykey ![username]@![hostname].local
```

now you should be able to retry the ssh command and see that it logs directly into your robot. 

Doubt: This doesn't seem to work 100% of the time. 



## Connect to the Internet

See: [](#duckiebot_network)



## Troubleshooting {#initialization-troubleshooting}

Symptom: The Pi hangs when you do `docker pull` commands or otherwise and sometimes shuts off

Resolution: An older version of the SD card image had the docker container `cjimti/iotwifi` running but this was found to be causing difficulties. SSH into your robot by some method and then execute:

```
duckiebot $ docker rm $(docker stop $(docker ps -a -q --filter ancestor=cjimti/iotwifi --format="{{.ID}}"))
duckiebot $ sudo systemctl unmask wpa_supplicant
duckiebot $ sudo systemctl restart networking.service
```



Symptom: You still have to enter your password when you login

Resolution:

TODO: Breandan Considine