# Troubleshooting {#troubleshooting status=beta}

<div class='requirements' markdown='1'>

Requires: The Raspberry Pi of the Duckiebot is connected to the battery.

Requires: The Stepper Motor HAT is connected to the battery.

Requires: You have a problem!

</div>



## Docker {#sec-docker-troubleshooting}

Symptom: `docker: Error response from daemon: Conflict. The container name "/![container_name]" is already in use by container "![container_hash]". You have to remove (or rename) that container to be able to reuse that name.`

Resolution: Stop (`docker stop ![container_name]`) if running and then remove (`docker rm ![container_name]`) the container with the  

## The Raspberry Pi does not turn ON

Symptom: The red LED on the Raspberry Pi is OFF

Resolution: Press the button on the side of the battery ([](#fig:troubleshooting-battery-button)).

<div figure-id="fig:troubleshooting-battery-button" figure-caption="The power button on the RAVPower Battery.">
     <img src="battery_button.jpg" style='width: 14em'/>
</div>


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


## The Duckiebot does not move {#troubleshooting-controller status=beta}

Symptom: I can SSH into my Duckiebot and run the joystick demo but the joystick does not move the wheels.

Resolution: Press the button on the side of the battery ([](#fig:troubleshooting-battery-button)).


Resolution: Check that the red indicator on the joystick stopped blinking.

<div figure-id="fig:joystick_connection_status" figure-class="flow-subfigures">
    <div figure-id="subfig:joystick_no_connection" figure-caption="Bad joystick status">
        <p style='width:14em'>
            <img src="joystick_no_connection.jpg" style='width:14em'/>
        </p>
    </div>

    <div figure-id="subfig:joystick_good_connection" figure-caption="Bad joystick status">
        <p style='width:14em'>
            <img src="joystick_good_connection.jpg" style='width:14em'/>
        </p>
    </div>
</div>

<!--
<img src="troubleshooting-images/joystick_no_connection.jpg" style='width:14em'/>
-->

Symptom: The joystick is connected (as shown in [](#subfig:joystick_good_connection)) but
the Duckiebot still does not move.

Resolution: Make sure that the controller is connected to the Duckiebot and that the OS
receives the data from it. Run

    duckiebot $ jstest /dev/input/js0

If you receive the error

    jstest: No such file or directory

it means that the USB receiver is not connected to the Raspberry Pi or is broken.
If the command above shows something like the following

    Driver version is 2.1.0.
    Joystick (ShanWan PC/PS3/Android) has 8 axes (X, Y, Z, Rz, Gas, Brake, Hat0X, Hat0Y)
    and 15 buttons (BtnX, BtnY, BtnZ, BtnTL, BtnTR, BtnTL2, BtnTR2, BtnSelect, BtnStart, BtnMode, BtnThumbL, BtnThumbR, ?, ?, ?).
    Testing ... (interrupt to exit)
    Axes:  0:     0  1:     0  2:     0  3:     0  4:-32767  5:-32767  6:     0  7:     0 Buttons:  0:off  1:off  2:off  3:off  4:off  5:off  6:off  7:off  8:off  9:off 10:off 11:off 12:off 13:off 14:off

it means that the USB receiver is connected to the Raspberry Pi. Leave the terminal above
open and use the joystick to command the Duckiebot. If you observe that the numbers shown
in the terminal change according to the commands sent through the joystick than the problem is
in ROS. Make sure that the joystick demo is launched. Restart the Duckiebot if needed and try
again.

If the numbers do not change while using the joystick then follow this guide at the next Resolution point.

Resolution: The controller might be connected to another Duckiebot nearby. Turn off the
controller, go to a room with no other Duckiebots around and turn the controller back
on. Retry.


## Hanging {#troubleshooting-hanging} 

Symptom: The Pi hangs when you do `docker pull` commands or otherwise and sometimes shuts off.

Resolution: An older version of the SD card image had the docker container `cjimti/iotwifi` running but this was found to be causing difficulties. SSH into your robot by some method and then execute:


    duckiebot $ docker rm $(docker stop $(docker ps -a -q --filter ancestor=cjimti/iotwifi --format="{{.ID}}"))
    duckiebot $ sudo systemctl unmask wpa_supplicant
    duckiebot $ sudo systemctl restart networking.service




Symptom: You still have to enter your password when you login

Resolution:

TODO: Breandan Considine
