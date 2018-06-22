# Networking aka the hardest part {#sec:duckiebot_network status=ready}

<div class='requirements' markdown="1">

Requires: A Duckiebot in configuration `DB17-CO+w`

Requires: Either a router that you have control over that has internet access, or your credentials for connecting to an existing wireless network

Requires: Patience (channel your inner Yoda)

Result: A Duckiebot that you can connect to and that is connected to the internet

</div>

Note: this page is primarily for folks operating with the "two-network" configuration, `C0+w`. For a one adapter setup you will can skip directly to [](#duckiebot-internet-access), but you will have to connect to a network that you can ssh through.

The basic idea is that we are going to use the "Edimax" thumbdrive adapter to create a dedicated wireless network that you can always connect to with your laptop. Then we are going to use the built-in Broadcom chip on the Pi to connect to the internet, and then the network will be bridged.

## (For `DB17-w`) Configure the robot-generated network

This part should work every time with very low uncertainty.

The Duckiebot in configuration `C0+w` can create a WiFi network.

It is a 5 GHz network; this means that you need to have a 5 GHz
WiFi adapter in your laptop.

First, make sure that the Edimax is correctly installed.
Using `iwconfig`, you should see four interfaces:

    duckiebot $ iwconfig
    wlx![AABBCCDDEEFFGG]  unassociated  Nickname:"rtl8822bu"

    ![...]

    lo        no wireless extensions.

    enxb827eb1f81a4  no wireless extensions.

    wlan1     IEEE 802.11bgn  ESSID:"duckietown"

    ![...]


Make note of the name `wlx![AABBCCDDEEFFGG]`.

Look up the MAC address using the command:

    duckiebot $ ifconfig wlx![AABBCCDDEEFFGG]
    wlx![AABBCCDDEEFFGG] Link encap:Ethernet  HWaddr ![AA:BB:CC:DD:EE:FF:GG]

Then, edit the connection file

    /etc/NetworkManager/system-connections/create-5ghz-network

Make the following changes:

* Where it says `interface-name=![...]`, put "`wlx![AABBCCDDEEFFGG]`".
* Where it says `mac-address=![...]`, put "`![AA:BB:CC:DD:EE:FF:GG]`".
* Where it says `ssid=duckiebot-not-configured`, put "`ssid=![robot name]`".

![Newly upgraded]

To ensure nobody piggybacks on our connection, which poses a security risk especially in a public environment, we will protect access to the 5 GHz WiFi through a password. To set a password you will need to log in the Duckiebot with the default "ubuntu" username and password and change your system files. In the `/etc/NetworkManager/system-connections/create-5ghz-network`, add:

    [wifi-security]
    key-mgmt=wpa-psk
    psk=YOUR_OWN_WIFI_PASSWORD_NO_QUOTATION_MAKRS_NEEDED
    auth-alg=open

and then reboot.

At this point you should see a new network being created named "`![robot name]`", protected by the password you just set.

Comment: Make sure the password contains min. 8 character or combined with numbers. If no networks shows up after the configuration and no feedback from system, please check the content of the file again. The program, which activates the Edimax wifi adapter is very sensitive to its content.   

<!--
If the Raspberry Pi's network interface is connected to the `duckietown` network
and to the internet, the Raspberry Pi will act as a bridge to the internet.
-->

<div class='only-zurich' markdown="1">
Adding a password to your 5GHz connection is a mandatory policy in the Zurich branch.
</div>

## Setting up wireless network configuration {#duckiebot-internet-access}


You are connected to the Duckiebot via WiFi, but the Duckiebot also needs to connect to the internet in order to get updates and install some software. This part is a little bit more of a "black art" since we cannot predict every possible network configurations. Below are some settings that have been verified to work in different situations:


### Option 1: `duckietown` WiFi

Check with your phone or laptop if there is a WiFi in reach with the name of `duckietown`. If there is, you are all set. The defaut configuration for the Duckiebot is to have one WiFi adapter connect to this network and the other broadcast the access point which you are currently connected to.

### Option 2.a): `eduroam` WiFi (Non-UdeM/McGill instructions) {status=draft}

If there should be no `duckietown` network in reach then you have to manually add a network configuration file for the network that you'd like to connect to. Most universities around the world should have to `eduroam` network available. You can use it for connecting your Duckiebot.

Save the following block as new file in `/etc/NetworkManager/system-connections/eduroam`:

    [connection]
    id=eduroam
    uuid=38ea363b-2db3-4849-a9a4-c2aa3236ae29
    type=wifi
    permissions=user:oem:;
    secondaries=

    [wifi]
    mac-address=![the MAC address of your internal wifi adapter, wlan0]
    mac-address-blacklist=
    mac-address-randomization=0
    mode=infrastructure
    seen-bssids=
    ssid=eduroam

    [wifi-security]
    auth-alg=open
    group=
    key-mgmt=wpa-eap
    pairwise=
    proto=

    [802-1x]
    altsubject-matches=
    eap=ttls;
    identity=![your eduroam username]@![your eduroam domain]
    password=![your eduroam password]
    phase2-altsubject-matches=
    phase2-auth=pap

    [ipv4]
    dns-search=
    method=auto

    [ipv6]
    addr-gen-mode=stable-privacy
    dns-search=
    method=auto

Set the permissions on the new file to 0600.

    sudo chmod 0600 /etc/NetworkManager/system-connections/eduroam

### Option 2.b): `eduroam` WiFi (UdeM/McGill instructions) {status=draft}

Save the following block as new file in `/etc/NetworkManager/system-connections/eduroam-![USERNAME]`:
where USERNAME is the your logged-in username in the duckiebot.

    [connection]
    id=eduroam
    uuid=38ea363b-2db3-4849-a9a4-c2aa3236ae29
    type=wifi
    permissions=user:![USERNAME]:;
    secondaries=

    [wifi]
    mac-address=![the MAC address of your internal wifi adapter, wlan0]
    mac-address-blacklist=
    mac-address-randomization=0
    mode=infrastructure
    seen-bssids=
    ssid=eduroam

    [wifi-security]
    auth-alg=open
    group=
    key-mgmt=wpa-eap
    pairwise=
    proto=

    [802-1x]
    altsubject-matches=
    eap=peap;
    identity=![DGTIC UNIP]
    password=![DGTIC PWD]
    phase2-altsubject-matches=
    phase2-auth=mschapv2

    [ipv4]
    dns-search=
    method=auto

    [ipv6]
    addr-gen-mode=stable-privacy
    dns-search=
    method=auto

Set the permissions on the new file to 0600.

    sudo chmod 0600 /etc/NetworkManager/system-connections/eduroam-![USERNAME]

### Option 3 (For Univeristé de Montréal students only): Use `UdeM avec cryptage` {status=draft}

TODO for Liam Paull: someone replicate please - LP

Note: your can use the `autoconnect-priority=XX` inside the `[connection]` block to establish a priority. If you want to connect to one network preferentially if two are available then give it a higher priority.

Save the following block as new file in `/etc/NetworkManager/system-connections/secure`:

    [connection]
    id=secure
    uuid=e9cef1bd-f6fb-4c5b-93cf-cca837ec35f2
    type=wifi
    permissions=
    secondaries=
    timestamp=1502254646
    autoconnect-priority=100

    [wifi]
    mac-address-blacklist=
    mac-address-randomization=0
    mode=infrastructure
    ssid=UdeM avec cryptage
    security=wifi-security

    [wifi-security]
    key-mgmt=wpa-eap

    [802-1x]
    eap=peap;
    identity=![DGTIC UNIP]
    phase2-auth=mschapv2
    password=![DGTIC PWD]

    [ipv4]
    dns-search=
    method=auto

    [ipv6]
    addr-gen-mode=stable-privacy
    dns-search=
    ip6-privacy=0
    method=auto

Set the permissions on the new file to 0600.

    sudo chmod 600 /etc/NetworkManager/system-connections/secure

### Option 4: custom WiFi {status=draft}

First run the following to see what networks are available:

    duckiebot $ nmcli dev wifi list

You should see the network that you are trying to connect (`![SSID])`) to and you should know the password. To connect to it run:

    duckiebot $ sudo nmcli dev wifi con ![SSID] password ![PASSWORD]

### Option 5: ETH Wifi {#wifi-ETH status=beta}

The following instructions will lead you to connect your PI to the "eth" wifi network.

First, run the following on duckiebot

    duckiebot $ iwconfig
    ![...]

    lo        no wireless extensions.

    enxb![xxxxxxxxxxx]  no wireless extensions.

    ![...]

Make note of the name `enxb![xxxxxxxxxxx]`. `![xxxxxxxxxxx]` should be a string that has 11 characters that is formed by numbers and lower case letters.

Second, edit the file `/etc/network/interfaces` which requires `sudo` so that it looks like the following, and make sure the `enxb![xxxxxxxxxxx]` matches.

Pay special attention on the line "pre-up wpa_supplicant -B -D wext -i wlan0 -c /etc/wpa_supplicant/wpa_supplicant.conf".This is expected to be exactly one line instead of two but due to formatting issue it is shown as two lines.

Also, make sure every characters match exactly with the provided ones. TAs will not help you to do spelling error check.

    # interfaces(5) file used by ifup(8) and ifdown(8) Include files from /etc/network/     interfaces.d:
    source-directory /etc/network/interfaces.d

    # The loopback network interface
    auto lo
    auto enxb![xxxxxxxxxxx]

    # the wired network setting
    iface enxb![xxxxxxxxxxx] inet dhcp

    # the wireless network setting
    auto wlan0
    allow-hotplug wlan0
    iface wlan0 inet dhcp
        pre-up wpa_supplicant -B -D wext -i wlan0 -c /etc/wpa_supplicant/wpa_supplicant.conf
        post-down killall -q wpa_supplicant

Third, edit the file `/etc/wpa_supplicant/wpa_supplicant.conf` which requires `sudo` so that it looks like the following, and make sure you substitute [identity] and [password] content with your eth account information:

    ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
    update_config=1

    network={
        ssid="eth"
        key_mgmt=WPA-EAP
        group=CCMP TKIP
        pairwise=CCMP TKIP
        eap=PEAP
        proto=RSN
        identity="your user name goes here"
        password="your password goes here"
        phase1="peaplabel=0"
        phase2="auth=MSCHAPV2"
        priority=1
    }

Fourth, reboot your PI.

    duckiebot $ sudo reboot

Then everything shall be fine. The PI will connect to "eth" automatically everytime it starts.

Note that, if something went wrong, your Duckiebot tries to connect to the network for 5.5mins at startup while it's blocking SSH connection to it completely ("Connection refused" error when connecting). If this is the case, please wait those 5.5mins until your Duckiebot lets you connect again and recheck your settings.

TODO: Find a solution to this since it occurs very often
