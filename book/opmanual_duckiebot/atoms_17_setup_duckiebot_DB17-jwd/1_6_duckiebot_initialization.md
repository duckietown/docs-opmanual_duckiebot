# Duckiebot Initialization {#setup-duckiebot status=beta}

Assigned: Andrea Censi


<div class='requirements' markdown="1">

Requires: An SD card of dimensions at least 16 GB.

Requires: A computer with an internet connection, an SD card reader, and 16 GB of free space.

Requires: An assembled Duckiebot in configuration `DB17`. This is the result of [](#assembling-duckiebot-db17-ttic).

Results: A Duckiebot that is configured correctly, that you can connect to with your laptop and hopefully also has internet access

</div>



## Acquire and burn the image {#setup-duckiebot-burn-image}

On the laptop, download the compressed image at this URL:

> [https://www.dropbox.com/s/8utdpl5lgi2pumo/duckiebot-RPI3-AD-2017-09-12.img.xz?dl=1](https://www.dropbox.com/s/8utdpl5lgi2pumo/duckiebot-RPI3-AD-2017-09-12.img.xz?dl=1)



The size is 1.7 GB.

You can use:

    $ wget -O duckiebot-RPI3-AD-2017-09-12.img.xz ![URL above]

<div class="comment" markdown="1">

The original was:

    $ curl -o duckiebot-RPI3-AD-2017-09-12.img.xz ![URL above]

It looks like that `curl` cannot be used with Drobpox links because it does not follow redirects.
</div>

To make sure that the image is downloaded correctly, compute its hash
using the program `sha256sum`:

    $ sha256sum duckiebot-RPI3-AD-2017-09-12.img.xz
    7136f9049b230de68e8b2d6df29ece844a3f830cc96014aaa92c6d3f247b6130  duckiebot-RPI3-AD-2017-09-12.img.xz

Compare the hash that you obtain with the hash above. If they are different,
there was some problem in downloading the image.

Uncompress the file:

    $ xz -d -k --verbose duckiebot-RPI3-AD-2017-09-12.img.xz


This will create a file of 11 GB in size.

Next, burn the image on disk.

See: The procedure of how to burn an image is explained in [](+software_reference#howto-burn-image).

## Turn on the Duckiebot

Put the SD Card in the Duckiebot.

Turn on the Duckiebot by connecting the power cable to the battery.

Comment: In general, for the battery: if it's off, a single click on the power button will turn the battery on. When it's on, a single click will show you the charge indicator (4 white lights = full), and holding the button for 3s will turn off the battery. Shutting down the Duckiebot is not recommended because it may cause corruption of the SD card.

## Connect the Duckiebot to a network

You can login to the Duckiebot in two ways:

1. Through an Ethernet cable.
2. Through a `duckietown` WiFi network.

In the worst case, you can use an HDMI monitor and a USB keyboard.

### Option 1: Ethernet cable

Connect the Duckiebot and your laptop to the same network
switch.

Allow 30 s - 1 minute for the DHCP to work.

### Option 2: Duckietown network

The Duckiebot connects automatically to a 2.4 GHz network
called "`duckietown`" and password "`quackquack`".

Connect your laptop to the same wireless network.


## Ping the Duckiebot

To test that the Duckiebot is connected, try to ping it.

The hostname of a freshly-installed duckiebot is `duckiebot-not-configured`:

    laptop $ ping duckiebot-not-configured.local

You should see output similar to the following:

    PING duckiebot-not-configured.local (![X.X.X.X]): 56 data bytes
    64 bytes from ![X.X.X.X]: icmp_seq=0 ttl=64 time=2.164 ms
    64 bytes from ![X.X.X.X]: icmp_seq=1 ttl=64 time=2.303 ms
    ![...]

## SSH to the Duckiebot

Next, try to log in using SSH, with account `ubuntu`:

    laptop $ ssh ubuntu@duckiebot-not-configured.local

The password is `ubuntu`.

By default, the robot boots into Byobu.

Please see [](+software_reference#byobu) for an introduction to Byobu.

Doubt: Not sure it's a good idea to boot into Byobu. -??

## Setup network

See: [](#duckiebot_network)

## Update the system

Next, we need to update to bring the system up to date.

Use these commands

    duckiebot $ sudo apt update
    duckiebot $ sudo apt dist-upgrade

## Give a name to the Duckiebot

It is now time to give a name to the Duckiebot.

These are the criteria:

- It should be a simple alphabetic string (no numbers or other characters like "`-`", "`_`", etc.) .
- It will always appear lowercase.
- It cannot be a generic name like "`duckiebot`", "`robot`" or similar.

From here on, we will refer to this string as "`![robot name]`".
Every time you see `![robot name]`, you should substitute the name that you chose.


## Change the hostname

We will put the robot name in configuration files.

Note: Files in `/etc` are only writable by `root`,
so you need to use `sudo` to edit them. For example:

    duckiebot $ sudo vi ![filename]

Edit the file

    /etc/hostname

and put "`![robot name]`" instead of `duckiebot-not-configured`.

Also edit the file

    /etc/hosts

and put "`![robot name]`" where `duckiebot-not-configured` appears.

The first two lines of `/etc/hosts` should be:

    127.0.0.1   localhost
    127.0.1.1   ![robot name]

Note: there is a command `hostname` that promises to change the hostname.
However, the change given by that command does not persist across reboots. You
need to edit the files above for the changes to persist.

Note: Never add other hostnames in `/etc/hosts`. It is a tempting
fix when DNS does not work, but it will cause other problems
subsequently.

Then reboot the Raspberry Pi using the command

    $ sudo reboot

After reboot, log in again, and run the command `hostname` to check that the
change has persisted:

    $ hostname
    ![robot name]

## Expand your filesystem {#expand-filesystem}

If your SD card is larger than the image, you'll want to expand the filesystem on your robot so that
you can use all of the space available. Achieve this with:

    duckiebot $ sudo raspi-config --expand-rootfs

and then reboot

    duckiebot $ sudo shutdown -r now

once rebooted you can test whether this was successful by doing

    duckiebot $ df -lh

the output should give you something like:

```
Filesystem      Size  Used Avail Use% Mounted on
/dev/root        15G  6.3G  8.2G  44% /
devtmpfs        303M     0  303M   0% /dev
tmpfs           431M     0  431M   0% /dev/shm
tmpfs           431M   12M  420M   3% /run
tmpfs           5.0M  4.0K  5.0M   1% /run/lock
tmpfs           431M     0  431M   0% /sys/fs/cgroup
/dev/mmcblk0p1   63M   21M   43M  34% /boot
tmpfs            87M     0   87M   0% /run/user/1000
```
You should see that the Size of your `/dev/root` Filesystem is "close" to the size of your SD card.


## Create your user {#create-user-on-duckiebot}

You must not use the `ubuntu` user for development.
Instead, you need to create a new user.

Choose a user name, which we will refer to as `![username]`.

To create a new user:

    duckiebot $ sudo useradd -m ![username]

Make the user an administrator by adding it to the group `sudo`:

    duckiebot $ sudo adduser ![username] sudo

Make the user a member of the groups `input`, `video`, and `i2c`

    duckiebot $ sudo adduser ![username] input
    duckiebot $ sudo adduser ![username] video
    duckiebot $ sudo adduser ![username] i2c

Set the shell `bash`:

    duckiebot $ sudo chsh -s /bin/bash ![username]


To set a password, use:

    duckiebot $ sudo passwd ![username]

At this point, you should be able to login to the new user from the laptop
using the password:

    laptop $ ssh ![username]@![robot name]

Next, you should repeat some steps that we already described.

Comment: What steps?? -LP

### Basic SSH config

Do the basic SSH config.

See: The procedure is documented in [](+software_reference#ssh-local-configuration).

### Create key pair

Next, create a private/public key pair for the user; call it `![username]@![robot name]`.

See: The procedure is documented in [](+software_reference#howto-create-key-pair).

### Add SSH alias {#setup-ssh-alias}

Once you have your SSH key pair on both your laptop and your Duckiebot, as well as your new user- and hostname set up on your Duckiebot, then you should set up an SSH alias as described in [](#ssh-aliases). This allows your to log in for example with

    laptop $ ssh ![abc]

instead of

    laptop $ ssh ![username]@![robot name]

where you can chose `![abc]` to be any alias / shortcut.

### Add the public key to Github

Add the public key to your Github account.

See: The procedure is documented in [](+software_reference#howto-add-pubkey-to-github).

If the step is done correctly, the following command should succeed and give you a welcome message:

    duckiebot $ ssh -T git@github.com
    Hi ![username]! You've successfully authenticated, but GitHub does not provide shell access.

### Local Git configuration

See: This procedure is in [](+software_reference#howto-git-local-config).

### Set up the laptop-Duckiebot connection

Make sure that you can login passwordlessly to your user from the laptop.

See: The procedure is explained in [](+software_reference#howto-login-without-password).
In this case, we have:
 `![local]` = laptop, `![local-user]` = your local user on the laptop,
 `![remote]` = `![robot name]`, `![remote-user]` = `![username]`.

If the step is done correctly, you should be able to login from the laptop to
the robot, without typing a password:

    laptop $ ssh ![username]@![robot name]

### Some advice on the importance of passwordless access

In general, if you find yourself:

- typing an IP
- typing a password
- typing `ssh` more than once
- using a screen / USB keyboard

it means you should learn more about Linux and networks, and you are setting
yourself up for failure.

Yes, you "can do without", but with an additional 30 seconds of your time. The
30 seconds you are not saving every time are the difference between being
productive roboticists and going crazy.

Really, it is impossible to do robotics when you have to think about IPs and
passwords...

## Other customizations

If you know what you are doing, you are welcome to install and use additional
shells, but please keep Bash as be the default shell. This is
important for ROS installation.

For the record, our favorite shell is ZSH with `oh-my-zsh`.

## Hardware check: camera {#camera-hardware-check}

Check that the camera is connected using this command:

    duckiebot $ vcgencmd get_camera
    supported=1 detected=1

If you see `detected=0`, it means that the hardware connection is not working.

You can test the camera right away using a command-line utility
called `raspistill`.

Use the `raspistill` command to capture the file `out.jpg`:

    duckiebot $ raspistill -t 1 -o out.jpg

Then download `out.jpg` to your computer using `scp` for inspection.

See: For instructions on how to use `scp`, see [](+software_reference#howto-download-file-with-scp).

## Final touches: duckie logo {#installing-duckietown-logo}

In order to show that your Duckiebot is ready for the task of driving around happy little duckies, the robot has to fly the Duckietown flag. When you are still logged in to the Duckiebot you can download and install the banner like this:

Download the ANSI art file from Github:

    duckiebot $ wget --no-check-certificate -O duckie.art "https://raw.githubusercontent.com/duckietown/Software/master/misc/duckie.art"

(optional) If you want, you can preview the logo by just outputting it onto the command line:

    duckiebot $ cat duckie.art

Next up create a new empty text file in your favorite editor and add the code for showing your duckie pride:

Let's say I use `nano`, I open a new file:

    duckiebot $ nano 20-duckie

And in there I add the following code (which by itself just prints the duckie logo):

    #!/bin/sh
    printf "\n$(cat /etc/update-motd.d/duckie.art)\n"

Then save and close the file. Finally you have to make this file executable...

    duckiebot $ chmod +x 20-duckie

...and copy both the duckie logo and the script into a specific directory `/etc/update-motd.d` to make it appear when you login via SSH. `motd` stands for "message of the day". This is a mechanism for system administrators to show users news and messages when they login. Every executable script in this directory which has a filename a la `![NN]-![some name]` will get exected when a user logs in, where `![NN]` is a two digit number that indicates the order.

    sudo cp duckie.art /etc/update-motd.d
    sudo cp 20-duckie /etc/update-motd.d

Finally log out of SSH via `exit` and log back in to see duckie goodness.

### Troubleshooting

Symptom: `detected=0`

Resolution: If you see `detected=0`, it is likely that the camera is not connected correctly.

If you see an error that starts like this:

    mmal: Cannot read camera info, keeping the defaults for OV5647
    ![...]
    mmal: Camera is not detected. Please check carefully the camera module is installed correctly.

then, just like it says: "Please check carefully the camera module is installed correctly.".

Symptom: random `wget`, `curl`, `git`, and `apt` calls fail with SSL errors.

Resolution: That's probably actually an issue with your system time. Type the command `timedatectl` into a terminal, hit enter and see if the time is off. If it is, you might want to follow the intructions from [this article][art1],
or entirely [uninstall your NTP service and manually grab the time on reboot][art2]. It's a bit dirty, but works surprisingly well.

[art1]: https://raspberrypi.stackexchange.com/questions/59860/time-and-timezone-issues-on-pi
[art2]: https://unix.stackexchange.com/questions/251519/setting-time-and-date-without-using-ntp

Symptom: Cannot find `/etc` folder for configuring the Wi-Fi. I only see `Desktop`, `Downloads` when starting up the Duckiebot.

Resolution: If a directory name starts with `/`, it's not supposed to be in the home directory, but rather at the root of the filesystem. You are currently in `/home/ubuntu`. Type `ls /` to see the folders at the root, including `/etc.
