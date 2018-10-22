# Network troubleshooting {#setup-troubleshooting-network status=draft}

Assigned: Manish

All errors that prevent the Duckiebot to answer to pings and SSH.


## I cannot ping the Duckiebot {status=draft}

Sympton: ping ![robot_name] does not work.

IMP: Check if you and duciebot intent to connect to the same network.

You can derive conclusions by following next few steps, and decide based upon it:

Step 1: Check that your rpi is responsive. Check for the led blinking on the rpi.

Step 2: Connect your duckiebot with the laptop using the ethernet cable. Check if you are able to ping the robot. This will provide you an hint if there is an issue with the robot or wifi.

Step 3: Check that this file: `/etc/wpa_supplicant/wpa_supplicant.conf` contains all the wifi networks in the coorect syntax that you want to connect.

Step 4: If it's your private access point, then you can access it using 192.168.0.1, where you can see all the devices coonected to the router. Make sure that both your duckiebot and your laptop are in the list.

Step 5: Check the file `~/.ssh/config` has the correct name hostname with hostname.local defined. 

## I cannot access my Duckiebot via SSH {#troubleshooting-mdns-ipv6 status=beta}

Symptom: When I run `ssh ![robot_name].local` I get the error `ssh: Could not resolve hostname ![robot_name].local`.

Resolution: Make sure that your Duckiebot is ON. Connect it to a monitor, a mouse and a keyboard. Run the command

    duckiebot $ sudo service avahi-daemon status

You should get something like the following

    ● avahi-daemon.service - Avahi mDNS/DNS-SD Stack
       Loaded: loaded (/lib/systemd/system/avahi-daemon.service; enabled; vendor preset: enabled)
       Active: active (running) since Sun 2017-10-22 00:07:53 CEST; 1 day 3h ago
     Main PID: 699 (avahi-daemon)
       Status: "avahi-daemon 0.6.32-rc starting up."
       CGroup: /system.slice/avahi-daemon.service
               ├─699 avahi-daemon: running [![robot_name_in_avahi].local
               └─727 avahi-daemon: chroot helpe

Avahi is the module that in Ubuntu implements the mDNS responder. The mDNS responder is responsible for advertising the hostname of the Duckiebot on the network so that everybody
else within the same network can run the command `ping ![robot_name].local` and reach your Duckiebot. Focus on the line containing the hostname published by the `avahi-daemon` on the network (i.e., the line that contains `![robot_name_in_avahi].local`).
If `![robot_name_in_avahi]` matches the `![robot_name]`, go to the next Resolution point.
If `![robot_name_in_avahi]` has the form `![robot_name]-XX`, where `XX` can be any number,
modify the file `/etc/avahi/avahi-daemon.conf` as shown below.

Identify the line

    use-ipv6=yes

and change it to

    use-ipv6=no

Identify the line

    #publish-aaaa-on-ipv4=yes

and change it to

    publish-aaaa-on-ipv4=no

Restart Avahi by running the command

    duckiebot $ sudo service avahi-daemon restart


## I can SSH to the Duckiebot but not without a password {status=draft}

Check the file ~.ssh/config, make sure you add your ssh key there, incase it doesn't exists.

The `init_sd_card` should generate a para in the above file in the following format:

# --- init_sd_card generated ---
Host duckiebot
    User duckie
    Hostname duckiebot.local
    IdentityFile /home/user/.ssh/DT18_key_00
    StrictHostKeyChecking no
# ------------------------------ 

       ssh-keygen -f "/home/user/.ssh/known_hosts" -R hostname.local
       
It will generate a key for you, if it doesn't exists.
