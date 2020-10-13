# Duckiebot SD Card Initialization {#setup-duckiebot status=ready}

This page is for the `DB18` configuration and above (including Jetson Nano configurations). 

<div class='requirements' markdown="1">

Requires: An SD card of size at least 32 GB. 

Requires: A computer with a **Ubuntu OS** (for flashing the SD card), an internet connection, an SD card reader, and 16 GB of free space.

Requires: Duckietown Shell, Docker, etc, as configured in [](#laptop-setup).

Requires: Duckietown Token set up as in [](#dt-account).

Results: A correctly configured Duckiebot SD card in configuration `DB18`. After assembling the Duckiebot, this will allow you to start it, connect to the internet, and get going.

</div>

## Choose a name for your robot {#chose-robot-hostname}

Pick a `hostname` for your robot. This will be the name of your robot and has to be unique within
a fleet of robots connected to the same network.
A valid `hostname` satisfies all the following requirements:

- it is lowercase
- it starts with a letter
- it contains only letters, numbers, and underscores

## Burn the SD card {#burn-sd-card}

Warning: this currently only works on Ubuntu. Mac is not supported.

Warning: on Ubuntu 16, you need to remove and re-insert the SD card. On Ubuntu 18 or above this is not necessary.

Plug the SD card in the computer using the card reader.

Warning: If your SD card have write protection switch on the side of the SD card, make sure it is set to write mode.

Then initalize it by running the command:

    laptop $ dts init_sd_card --hostname ![hostname] [options]

The basic options are:

    --hostname         Hostname of the device to flash. This is required.
    --wifi             default: duckietown:quackquack
    --country          default: US
    --type             The type of your device. Types are `duckiebot` (default), 
                       `watchtower`, `traffic_light`.
    --configuration    The configuration of your robot. This is associated with 
                       `--type` option. E.g. `DB-beta`, `DB20`, `DB19`, or `DB18`.

Note: the default username and password for all duckiebots are "duckie" and "quackquack", respectively. 

If you plan on connecting with the Duckiebot over different networks (e.g. at home and in class), you can list them like that (note there should be no space after the commas):

    laptop $ dts init_sd_card --hostname ![hostname] --wifi duckietown:quackquack,myhomenetwork:myhomepassword,myuninetwork:myunipassword

Default for watchtower and traffic_light is no wifi config. Default for other robot types is "duckietown:quackquack" Each network defined in the list can have between 1 and 3 arguments:

      - Open networks (no password) network: "ssid"
      - PSK (Pre-shared key) protected networks (no password) network: "ssid:psk"
      - EAP (Extensible Authentication Protocol) protected networks network: "ssid:username:password"

If you want to add additional networks later and you have to edit the `/etc/wpa_supplicant/wpa_supplicant.conf` file in the `root` drive (for Raspberry Pi), or the `/etc/wpa_supplicant.conf` in the `root` drive (for the Jetson Nano board).

New networks can be created by adding a new `network={}` paragraph, and then entering the network information. An example network configuration is shown below:

```
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1
country=CH

network={
    id_str="network_1"
    ssid="comnet23243"
    psk="MSNDJWKE32"
    key_mgmt=WPA-PSK
}

network={
    id_str="network_2"
    ssid="TPlink23432"
    psk="ksnbn4wn3"
    key_mgmt=WPA-PSK
}
```

Make sure to set your country correctly with the `--country` option. (Ex. CA for Canada, CH for Switzerland) This sometimes will result in the specific Wifi hotspot not being seen on the duckiebot problem.

Additional options for init_sd_card are provided, however, it is recommended that you only use those if you know what you are doing:

    --no-cache         Uses "fresh" image instead of the cached one.
    --workdir          Temporary working directory.
    --device           The device you want to flash the image to
    --steps            Steps to perform
    --linux-username   Username of the linux user to create on the flashed
                       device The default is: duckie
    --linux-password   Password to access the linux user profile created on
                       the flashed device The default is: quackquack    

For a full list of the options, run

    laptop $ dts init_sd_card --help

Example initialization for the `DB-beta` using wifi network "duckienet" with password "quackquack". 
    
    laptop $ dts init_sd_card --type duckiebot --configuration DB-beta --country CH --hostname studentduck --wifi duckienet:quackquack

After you run the `dts init_sd_card` command with your options follow the instructions that appear on screen. Select the drive with the correct size (usually `/dev/mmcblk` or `/dev/sdc`) by pressing <kbd>Enter</kbd>.

Note: If you don't know where to find the drive path, you can utilize the command line `lsblk`. This command should show you all the disks on the machine.

```
NAME   MAJ:MIN RM   SIZE RO TYPE MOUNTPOINT
loop0    7:0    0   2.3M  1 loop /snap/gnome-calculator/260
loop1    7:1    0  14.5M  1 loop /snap/gnome-logs/45
loop2    7:2    0 140.7M  1 loop /snap/gnome-3-26-1604/74
loop3    7:3    0    91M  1 loop /snap/core/6350
loop4    7:4    0  34.6M  1 loop /snap/gtk-common-themes/818
loop5    7:5    0   3.7M  1 loop /snap/gnome-system-monitor/57
loop6    7:6    0    13M  1 loop /snap/gnome-characters/139
sda      8:0    0 298.1G  0 disk
└─sda1   8:1    0 298.1G  0 part /
sdb      8:16   1  29.1G  0 disk
├─sdb1   8:17   1    64M  0 part
└─sdb2   8:18   1  29.1G  0 part
sr0     11:0    1  1024M  0 rom

```

Using above listing as an example, you should be choosing the disk name (sdb), not the partition name (sdb1, sdb2) for etcher to capture the whole disk.

For Raspberry Pi:

- You will then have to enter your laptop's `sudo` password to run Etcher.

- When asked "Are you sure?" select <kbd>y</kbd>.

Warning: Always be careful when selecting disk to be imaged. You don't want to lose your computer's system partition!

For Jetson Nano:

- The procedure will ask to accept the conditions for use. When asked "Do you accept? (a=Accept, n=Reject, r=Read License) \[n]: " select <kbd>r</kbd> to read the license, and then <kbd>a</kbd> to accept.

- After the image is downloaded, you will have to enter your `sudo` password. 

- For the Jetson Nano board, the drive selection (e.g. /dev/sda) is performed after the image is downloaded. 

On successful end of the procedure, the drives will be automatically ejected and you can just remove the SD card from your laptop.

### Troubleshooting

Symptom: The SD card doesn't seem to be written.

Symptom: The SD card process seems extremely fast, and there is no data on my SD card.

Resolution: Check if your SD card is good, and check if it has a write protection switch and it is in read-only mode.

Symptom: On Ubuntu 16, it prompts with errors about directories not mounted

Resolution: If the procedure fails with errors about directories not mounted, be patient and do it again, this time leaving the SD card in.

Symptom: The flashing procedure failes with a `Bad archive` error when trying to flash the Hypriot image

Resolution: This happens when the downloaded zip for Hypriot is incomplete or corrupt. Delete the zip file by running the following command and try the procedure again. Also check if your computer has enough storage space.

    laptop $ rm /tmp/duckietown/hypriotos*
    
Symptom: The verification process fails with error `Please set up a token using "dts tok set"`.

Resolution: Redo the Duckietown Token setup procedure [](#dt-account).

## Booting the Duckiebot {#duckiebot-boot}

Now insert the SD card into the Raspberry PI (or Jetson Nano for `DB-beta`) and push the button on the battery to power things up.

Warning: Don't charge the battery while you are doing the initialization (or in general when the Duckiebot is turned on). The external power supply might not be able to provide sufficient current and the Raspberry Pi will reboot. Should that happen during the initialization procedure, you will likely have to burn the SD card again.

## Monitoring the First Boot {#monitor-first-boot}

You know that your Raspberry Pi, or Jetson Nano, has successfully booted when you see it using the `dts fleet discover` utility:

Open a terminal and run the command

```
laptop $ dts fleet discover
```
For the Jetson Nano board, the first boot of the Duckiebot will take several minutes, and then it will reboot automatically. Only after it reboots you will be able to ssh into the bot. This can be monitored using an external monitor, or by running the fleet discover command after successful rebooting. 

Note: If the command above returns an error about the library `zeroconf` being
missing, run `pip3 install zeroconf` and retry.

The command above (`fleet discover`) will show a list of all the duckiebots
reachable on your network. Leave this tool open, it will refresh automatically every
second, so there is no need to manually restart it. You should see your duckiebot in a few minutes after you inserted your SD card and power on the robot.

The list will look like the following.

<div figure-id="fig:fleet-discover" figure-caption="Output of 'dts fleet discover'">
     <img src="fleet_discover.jpg" style='width: 34em'/>
</div>

After you turn your Duckiebot ON with a new SD card in, you will see your
Duckiebot appear in the list within approximately 2 minutes.
The column you will need to monitor is _Dashboard_. Wait until the
Dashboard is _Up_ as shown in the image below.

<div figure-id="fig:fleet-discover-dashboard-up" figure-caption="Output of 'dts fleet discover' (Dashboard Up)">
     <img src="fleet_discover_dashboard_up.jpg" style='width: 34em'/>
</div>

When the Dashboard is Up, open your browser and visit the
URL `http://![hostname].local/`. You will see the following page,

<div figure-id="fig:compose_first_setup" figure-caption="">
  <img src="compose_first_setup.png" style='width: 34em'/>
</div>

This is the dashboard of your Duckiebot. The Dashboard is built using a
framework called \\compose\\. You will see how to configure it in [](#duckiebot-dashboard-setup)

Do not power the robot off (by holding the battery button) after you have booted the robot the first time but before you see it show up in `dts fleet discover`.

### Troubleshooting for first boot

Symptom: The red LED on the Raspberry Pi is OFF.

Resolution: Press the button on the side of the battery ([](#troubleshooting-battery-button)).

<figure id="troubleshooting-battery-button">
    <figcaption>The power button on the RAVPower Battery.</figcaption>
     <img src="battery_button.jpg" style='width: 14em'/>
</figure>

Symptom: The Raspberry Pi has power but it does not boot.

Resolution: [Initialize the SD card](#setup-duckiebot) if not done already. If problem persists, try again.

Symptom: I cannot ping the duckiebot.

Resolution: Check the [networking section](#duckiebot-network) of the book to see if your networking is good! 

Symptom: I am not sure whether the Duckiebot is properly initialized.

Resolution: As long as the fleet discover tool shows ready, your duckiebot should be ready. You can also visit `http://![hostname].local:9000` to see all the container status. Generally as long as you see the duckiebot web UI is up, your duckiebot should be good.

Symptom: The LEDs light up in a variety of colors when the battery is plugged in for the first time.

Resolution: The LEDs of the Duckiebot should light up in white as soon as you power the Duckiebot. If the LEDs turn on and shine in any different color than white, probably the code on the microcontroller is corrupted. You can reflash it using the procedure in [](#reflash-microcontroller).

## SSH to the Duckiebot {#setup-duckiebot-ssh}

Next, try to log in using SSH, using

    laptop $ ssh duckie@![hostname].local

This should succeed without password. Note that duckie is the default user. If you modified the username during the SD card initialization procedure, then use the appropriate username here.

## Rebooting the Duckiebot {#setup-duckiebot-reboot}

Warning: Do not test these commands now if you just booted up your duckiebot for the first time. It is likely not finished initializing and shutting down the duckiebot or disconnecting its internet access could interrupt the process and require you to re-flash the SD card.

To reboot:

    laptop $ ssh duckie@![hostname].local sudo reboot

## Turn off the Duckiebot {#setup-duckiebot-poweroff}

To turn off the Duckiebot, use:

    laptop $ ssh duckie@![hostname].local sudo poweroff

Then wait 30 seconds.

Warning: If you disconnect the power before shutting down properly using `shutdown`,
the system might get corrupted.

Then disconnect the USB cable (from the large connector next to the battery).

Warning: If you disconnect frequently the cable at the Raspberry Pi's end, you might damage the port.

Warning: Pressing the battery button does not shut down the power to the Duckiebot, it only activates the battery. If not in use anymore, disconnect the cables. The battery will automatically shut down if no load is detected over a period of 10 mins. 
