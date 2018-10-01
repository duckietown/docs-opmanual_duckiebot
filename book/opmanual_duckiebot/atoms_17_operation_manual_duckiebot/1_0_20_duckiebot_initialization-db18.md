# Duckiebot Initialization {#setup-duckiebot status=ready}

Assigned: Breandan Considine, Liam Paull

This page is for the `DB18` configuration used in classes in 2018. For last year's instructions see [here](https://docs.duckietown.org/DT17/). 

<div class='requirements' markdown="1">

Requires: An SD card of size at least 16 GB.

Requires: A computer with a **Ubuntu OS** (for flashing the SD card), an internet connection, an SD card reader, and 16 GB of free space.

Requires: Duckietown Shell, Docker, etc, as configured in [](#laptop-setup).

Requires: Duckietown Token set up as in [](#dt-account).

Results: A correctly configured Duckiebot SD card in configuration `DB18`. After assembling the Duckiebot, this will allow you to start it, connect to the internet, and get going.


</div>


## Burn the SD card {#burn-sd-card}

Warning: this currently only works on Ubuntu. Mac is not supported.

Plug the SD card in the computer using the card reader. 

Then initalize it by running the command:


    laptop $ dts init_sd_card [options]

The options are:

    --hostname         default: duckiebot
    --wifi-ssid        default: duckietown
    --wifi-password    default: quackquack
    --linux-username   default: duckie
    --linux-password   default: quackquack
    
For example, if your home network is "mynetwork" with password "mypassword", and you want to call your duckiebot "mybot", use:

    laptop $ dts init_sd_card --wifi-ssid mynetwork --wifi-password mypassword --hostname mybot


Then follow the instructions that appear on screen:


- You will then have to enter your laptop's `sudo` password to run Etcher.

- Select the drive at `/dev/mmcblk0` by pressing <kbd>Enter</kbd>.

- When asked "Are you sure?" select <kbd>y</kbd>.

When the SD card is completely written, you should arrive at `Press any key to continue`. Do so and the script will exit. 


Note: on Ubuntu 16, you need to remove and re-insert the SD card. On Ubuntu 18 this is not necessary.

If the procedure fails with errors about directories not mounted, be patient and do it again, this time leaving the SD card in.

If you plan on connecting with the Duckiebot over different networks (e.g. at home and in class), you can add their details before removing the SD card from your laptop. The SD card writing procedure should have created two new drives on your computer: `root` and `HyperiotOS`. In `HypriotOS` edit the file `user-data`. Find the lines with the SSID and password that you filled in already: 
``` 
- content: |
      country=CA
      ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
      update_config=1
      network={
          ssid="![wifi-ssid]"
          psk="![wifi-password]"
          key_mgmt=WPA-PSK
      }
    path: /etc/wpa_supplicant/wpa_supplicant.conf
```
You can add as many network configurations as you want. If you expect that two networks will be available at the same time, you can give them priority (a higher value means a network is more desirable):
``` 
- content: |
      country=CA
      ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
      update_config=1
      network={
          ssid="![wifi-ssid]"
          psk="![wifi-password]"
          key_mgmt=WPA-PSK
          priority=1
      }
      network={
          ssid="![duckietown-wifi-ssid]"
          psk="![duckietown-wifi-password]"
          key_mgmt=WPA-PSK
          priority=3
      }
      network={
          ssid="![home-wifi-ssid]"
          psk="![home-wifi-password]"
          key_mgmt=WPA-PSK
          priority=2
      }
    path: /etc/wpa_supplicant/wpa_supplicant.conf
```

You can then eject (safe remove) the drives and remove the SD card from your laptop. 

Warning: this will work only if done **before the first boot**. If you want to add additional networks later and you have to edit  the `/etc/wpa_supplicant/wpa_supplicant.conf` file in the `root` drive.


## Booting the Duckiebot {#duckiebot-boot}

Now insert the SD card into the Raspberry PI and push the button on the battery to power things up. 

You should immediately see the **green** light next to where the SD card was inserted start to blink with activity. 

If not, stop, as there is a problem with the SD card (or possibly the PI, but this is unlikely).

Warning: Allow the robot time to boot. On first boot it may take up to 5 mins or more since some things are being configured. 

Do not power the robot off (by holding the battery button) while this is in process. 


You know that your Pi has successfully booted when you are able to ping your robot with the command bellow or with some method in [](#duckiebot-network):

```
laptop $ ping ![hostname].local
```

Note that you should be connected to the same network as the robot in order to do that. If you are using a virtual machine you should use Bridged connection (typically NAT is used by default).

You should see output similar to the following:​    

```
PING ![hostname].local (![X.X.X.X]): 56 data bytes
64 bytes from ![X.X.X.X]: icmp_seq=0 ttl=64 time=2.164 ms
64 bytes from ![X.X.X.X]: icmp_seq=1 ttl=64 time=2.303 ms
![...]
```



## SSH to the Duckiebot {#setup-duckiebot-ssh} 

Next, try to log in using SSH, using

    laptop $ ssh ![hostname]


This should succeed without password. 

If it doesn't work, check that `~/.ssh/config` contains something like:

    Host ![hostname]
        User duckie
        Hostname ![hostname].local
        IdentityFile ~/.ssh/DT18_key_00
    
This configuration was added by the `init_sd_card` command.


### Workarounds

It is recommended that you immediately give ownership of your home folder to your user. You can do this by running the following on your Duckiebot:


    duckiebot $ sudo chown -R ![username]:![username] .

## Rebooting the PI {#setup-duckiebot-reboot}


To reboot:

    laptop $ ssh ![hostname] sudo reboot
    
## Turn off the PI {#setup-duckiebot-poweroff}
   
To turn off the Duckiebot, use:

    laptop $ ssh ![hostname] sudo poweroff 
    
Then wait 30 seconds.


Warning: If you disconnect the power before shutting down properly using `shutdown`,
the system might get corrupted.

    
Then disconnect the USB cable (from the large connector next to the battery).

Warning: If you disconnect frequently the cable at the Raspberry Pi's end, you might damage the port.


