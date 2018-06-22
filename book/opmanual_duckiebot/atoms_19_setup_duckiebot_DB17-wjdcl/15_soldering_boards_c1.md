# Soldering boards (`DB17-l`) {#soldering-boards-c1 status=deprecated}

Note: General rule in soldering

* soldering the components according to the height of components - from lowest to highest

Assigned: Jacopo Tani

<div class='requirements' markdown="1">

Requires: Duckiebot `DB17-l` parts. The acquisition process is explained in [](#acquiring-parts-c1).
The configurations are described in [](#duckiebot-configurations).

Requires: Time: 30 minutes

Results: A `DB17-l` Duckiebot

</div>

## General rules

General rule in soldering:

* soldering the components according to the height of components - from lowest to highest

## 16-channel PWM/Servo HAT

([alternative instructions: how to solder on the PWM/Servo HAT](https://learn.adafruit.com/adafruit-16-channel-pwm-servo-hat-for-raspberry-pi/))


### Prepare the components

Put the following components on the table according the Figure

* [GPIO Stacking Header](http://adafru.it/2223) for A+/B+/Pi 2
* [Adafruit](http://adafru.it/2327) Mini Kit of 16-Channel PWM / Servo HAT for Raspberry Pi
    * 3x4 headers (4x)
    * 2-pin terminal block
    * 16-Channel PWM / Servo HAT for Raspberry Pi (1x)


<div figure-id="fig: " figure-caption=" ">
     <img src="image_3.jpg" style='width: 20ex; height: auto'/>
</div>

### Soldering instructions

1. Solder the 2 pin terminal block next to the power cable jack
2. Solder the four 3x4 headers onto the edge of the HAT, below the words "Servo/PWM Pi HAT!"
3. Solder the GPIO Stacking Header at the top of the board, where the 2x20 grid of holes is located.


## LSD board

TODO: add LSD board image, top and bottom.

<img src="image_5.png" style='width: 20em; height: auto'/>

<img src="image_6.png" style='width: 20em; height: auto'/>

### Prepare the components

Put the following components according the figure on the table:

* 1 x 40 pin female header
* 5 x 4 pin female header
* 2 x 16 pin male header
* 1 x 12 pin male header
* 1 x 3 pin male header
* 1 x 2 pin female shunt jumper
* 5 x 200 Ohm resistors
* 10 x 130 Ohm resistors
* 3 x 4 pin male header for servos

<div figure-id="fig: LSD_HAT_and_all_components" figure-caption="LSD HAT and all of needed components">
     <img src="LSD_HAT.jpg" style='width: 20em; height: auto'/>
</div>

### Soldering instructions

1. Put the resistors on the top of the board according to silkscreen markings, solder it on from the bottom side.

Tips:

1. Solder all female headers to the bottom of the board. Alignment becomes easy if the  female headers are plugged into the PWM heat, and the LSD board rests on top.

3. Solder all male headers to the top of the board. Male header positions are outlined on the silkscreen.


## LED connection

<img src="image_11.jpg" style='width: 20ex; height: auto'/>

<img src="image_12.jpg" style='width: 20ex; height: auto'/>


Parts list:

* 4 x 6" female-female jumper cable

Instructions:

1. Connect LED accordingly to silkscreen indication on PRi 2 LSD board

2. silkscreen legend: Rx, Gx, Bx are red, green, and blue channels, accordingly, where x is the LED number; C is a common line (either common anode or common cathode)

3. For adafruit LEDs are common anode type. The longest pin is common anode. Single pin on the side of common is red channel. The two other pins are Green and Blue channels, with the blue furthest from the common pin.

4. Both types of LEDs are supported. Use shunt jumper to select either common anode (CA) or common cathode (CC) on 3-pin male header. Note, however, that all LEDs on the board must be of the same type.


## Putting everything together!

1. Stack the boards

    1. Screw the first eight standoffs into the Pi - provide hints on the location of standoffs and the suggested orientation of the boards w/r to the chassis

    2. connect the camera to the Pi [image showing the connector ?]

    3. Stack the DC/Stepper Motor HAT onto the Pi, aligning both sets of GPIO pins over each other and screw the standoffs to secure it. Try to not bend the camera connector too much during this step

    4. Stack the 16-channel PWM/Servo HAT onto the Pi, both sets of GPIO pins over each other and screw the standoffs to secure it

2. Slide the battery between the two chassis plates

3. Power the PWM/Servo HAT and Pi connecting them to the battery with the cables included in the duckiebox

4. Power the DC/Stepper motor from the PWM/Servo HAT using the male-to-male cable in the duckiebox, connect the positive

5. connect the Pi to the board

6. Finished!
