# Assembling the Duckiebot (`DB17-jwd`)  {#assembling-duckiebot-c0 status=deprecated}

Point of contact: Shiying Li

Once you have received the parts and soldered the necessary components, it is time to assemble them in a Duckiebot. Here, we provide the assembly instructions for configurations `DB17-wjd`.

<div class='requirements' markdown="1">

Requires: Duckiebot `DB17-wjd` parts. The acquisition process is explained in [](#acquiring-parts-c0).

Requires: Having soldered the `DB17-wjd` parts. The soldering process is explained in [](#soldering-boards-c0).

Requires: Having prepared the power cable. The power cable preparation is explained in [](#power-cable-prep-c0). Note: Not necessary if you intend to build a `DB17-l` configuration.

Requires: Having installed the image on the MicroSD card. The instructions on how to reproduce the Duckiebot system image are in [](#duckiebot-ubuntu-image).

Requires: Time: about 40 minutes.

Results: An assembled Duckiebot in configuration `DB17-wjd`.

</div>

Note: The [FAQ](#op-assembly-db17jwd-faq) section at the bottom of this page may already answer some of you comments, questions or doubts.

Note: While assembling the Duckiebot, try to make as symmetric (along the longitudinal axis) as you can. It will help going forward.

## Chassis

Open the Magician chassis package ([](#fig:duckiebot_components)) and take out the following components:

* Chassis-bottom (1x), Chassis-up (1x);
* DC Motors (2x), motor holders (4x);
* Wheels (2x), steel omni-directional wheel (1x);
* All spacers and screws;
* Screwdriver.

<figure id="duckiebot_components" figure-caption="Components in Duckiebot package.">
     <img src="duckiebot_components.png" style='width: 30em'/>
</figure>

Note: You won't need the battery holder and speed board holder (on the right side in [](#fig:duckiebot_components)).

### Bottom

Insert the motor holders on the chassis-bottom and put the motors as shown in the figure below (with the longest screws (M3x30) and M3 nuts).

<figure id="motors" figure-caption=" Components for mounting the motor">
     <img src="motors.jpg" style='width: 30em'/>
</figure>

<figure id="scratch_motors" figure-caption="The scratch of assembling the motor">
     <img src="scratch_motors.png" style='width: 25em'/>
</figure>

<figure id="motors1" figure-caption="Assembled motor">
     <img src="motors1.jpg" style='width: 25em'/>
</figure>

Note: Orient the motors so that their wires are inwards, i.e., towards the center of the chassis-bottom. The black wires should be closer to the chassis-bottom to make wiring easier down the line.

Note: if your Magician Chassis package has unsoldered motor wires, you will have to solder them first. Check these instructions. In this case, your wires will not have the male pin headers on one end. Do not worry, you can still plug them in the stepper motor hat power terminals.

TODO: make instructions for soldering motor wires

### Wheels

Plug in the wheels to the motor as follows (no screws needed):

<figure id="horizontal">
    <figcaption>Wheel assembly instructions</figcaption>

    <div figure-id="fig:scratch_wheels" figure-caption="Wheel assembly schematics">
        <img src="scratch_wheels.png" style='width: 30em'/>
    </div>

    <div figure-id="fig:wheels" figure-caption="Assembled wheels">
        <img src="wheels.jpg" style='width: 25em'/>
    </div>
</figure>

### Omni-directional wheel

The Duckiebot is driven by controlling the wheels attached to the DC motors. Still, it requires a "passive" omni-directional wheel (the _caster_ wheel) on the back.

The Magician chassis package contains a steel omni-directional wheel, and the related standoffs and screws to secure it to the chassis-bottom part.

<figure id="scratch_omni" figure-caption="The omni-directional wheel schematics">
     <img src="scratch_omni.png" style='width: 30em'/>
</figure>

<figure id="omni" figure-caption="Assembled omni-directional wheel">
     <img src="omni.jpg" style='width: 30em'/>
</figure>


### Caster wheel {#caster-wheel-instruction}

As alternative to omnidirection wheel, caster wheel has less friction.
If you have purchased caster wheel, read this section.

To assemble the caster wheel, the following materials are needed:

* caster wheel (1x)
* Metal standoffs (M3x12mm F-F, 6mm diameter) (4x)
* Metal screws (M3x8mm) (8x)
* Split/Spring lock washers (M3) (8x)
* Flat lock washers (M3) (8x)

<figure id="components_caster" figure-caption="Component-List for assemblying the caster wheels">
     <img src="component_list_caster_wheels.jpg" style='width: 30em'/>
</figure>

#### Prepare the Screws with Washers

The lock washers belongs to screw-head side [](#fig:screws_washers_boltz), i.e. the split lock washer and the flat lock washers stays always near the screw head.
The split lock washer stays near the screw head. First split lock washer, then flat lock washer.

<figure id="screws_washers_boltz" figure-caption="Insert the locker washers into metal screws from left to right">
     <img src="screws_washer_boltz.jpg" style='width: 30em'/>
</figure>

<figure id="screw_washer" figure-caption="The metal screws with the lock washers">
     <img src="screw_washer.jpg" style='width: 30em'/>
</figure>

#### Assembly the metal standoffs on the caster wheels

Fasten the screws with washers on the caster wheels from the bottom up and screw the metal standoffs from top to bottom. The caster before mounting looks like in [](#fig:caster_sideview).


<figure id="caster_sideview" figure-caption="The assembled caster before mounting it under the chassis-bottom">
     <img src="caster_sideview.jpg" style='width: 30em'/>
</figure>

#### Assembly the caster wheels under the chassis bottom

Assembly the prepared caster wheels in the front side of duckiebot under the chassis bottom. Fasten the screws with washers from top to bottom.

Better: In order to get all the screws properly into the metal standoffs, let all the screws stay loose within the right positions before all the screws are inserted into the standoffs[](#fig:assembled_caster).

<figure id="caster_above" figure-caption="Assembly the caster wheels under chassis-bottom">
     <img src="caster_above.jpg" style='width: 30em'/>
</figure>

<figure id="assembled_caster" figure-caption="Assembled caster wheels (sideview)">
     <img src="assembled_caster_sideview.jpg" style='width: 30em'/>
</figure>


### Mounting the standoffs

Put the car upright (omni wheel pointing towards the table) and arrange wires so that they go through the center rectangle. Put 4 spacers with 4 of M3x6 screws on exact position of each corner as below [](#fig:assembled_spacers).


<figure id="screws" figure-caption="Metal spacers and M3x6mm screws">
     <img src="chassi_screws.jpg" style='width: 25em'/>
</figure>

<figure id="assembled_spacers" figure-caption="The spacers on each corner of the chassis-bottom">
     <img src="assembled_spacers.jpg" style='width: 25em'/>
</figure>


The bottom part of the Duckiebot's chassis is now ready. The next step is to assemble the Raspberry Pi on chassis-top part.


## Assembling the Raspberry Pi, camera, and HATs

### Raspberry Pi

Before attaching anything to the Raspberry Pi you should add the heat sinks to it. There are 2 small sinks and a big one. The big one best fits onto the processor (the big "Broadcom"-labeled chip in the center of the top of the Raspberry Pi). One of the small ones can be attached to the small chip that is right next to the Broadcom chip. The third heat sink is optional and can be attached to the chip on the underside of the Raspberry Pi. Note that the chip on the underside is bigger than the heat sink. Just mount the heat sink in the center and make sure all of them are attached tightly.

When this is done fasten the nylon standoffs on the Raspberry Pi, and secure them on the top of the chassis-up part by tightening the nuts on the opposite side of the chassis-up.


<figure id="Raspberry_pi3_parts" figure-caption="Components for Raspberry Pi3">
     <img src="RPi_3_parts.jpg" style='width: 30em'/>
</figure>

<figure id="Raspberry_pi3_heatsinks" figure-caption="Heat sink on Raspberry Pi3 ">
     <img src="RPi_3_heatsinks.jpg" style='width: 25em'/>
</figure>

<figure id="SideView_Raspberry_pi3" figure-caption="Nylon standoffs for Raspberry Pi3">
     <img src="side_RPi_3.jpg" style='width: 25em'/>
</figure>

<figure id="raspi_chassis_bottom" figure-caption="Attach the nylon huts for the standoffs (bottom view)">
     <img src="raspi_chassis_bottom.jpg" style='width: 25em'/>
</figure>

<figure id="raspi_chassis_up" figure-caption="Assembled Raspberry Pi3 (top view) ">
     <img src="raspi_chassis_up.jpg" style='width: 25em'/>
</figure>

### Micro SD card

Requires: Having the Duckiebot image copied in the micro SD card.

Take the micro SD card from the Duckiebox and insert its slot on the Raspberry Pi. The SD card slot is just under the display port, on the short side of the PI, on the flip side of where the header pins are.

<figure id="SD_card" figure-caption="The micro SD card and mirco SD card readers">
    <img src="sd_card.jpg" style='width: 25em'/>
</figure>

<figure id="RASPI_SD" figure-caption="Inserted SD card">
    <img src="pi-with-sdcard.jpg" style='width: 30em'/>
</figure>

### Camera

Note: If you have camera cables of different lengths available, keep in mind that both are going to work. We suggest to use the longer one, and wrap the extra length under the Raspberry Pi stack.

#### The Raspberry Pi end

First, identify the camera cable port on the Pi (between HDMI and power ports) and remove the orange plastic protection (it will be there if the Pi is new) from it. Then, grab the long camera cable (300 mm) and insert in the camera port. To do so, you will need to gently pull up on the black connector (it will slide up) to allow the cable to insert the port. Slide the connector back down to lock the cable in place, making sure it “clicks”.

TODO: insert image with long cable

<figure id="raspi_camera_apart" figure-caption="Camera port on the Raspberry Pi and camera cable ">
     <img src="raspi_camera_apart.jpg" style='width: 25em'/>
</figure>

Note: Make sure the camera cable is inserted in the right direction! The metal pins of the cable should be in contact with the metal terminals in the camera port of the PI.


<figure id="camera_with_long_cable" figure-caption="Camera with long cable">
     <img src="ziptied_top_camera.jpg" style='width: 25em'/>
</figure>

#### The camera end

If you have the long camera cable, the first thing to do is removing the shorter cable that comes with the camera package. Make sure to slide up the black connectors of the camera-camera cable port in order to unblock the cable.

Take the rear part of the camera mount and use it hold the camera in place. Note that the camera is just press-fitted to the camera mount, no screws/nuts are needed.

In case you have not purchased the long camera cable, do not worry! It is still very possible to get a working configuration, but you will have little wiggling space and assembly will be a little harder.

Place the camera on the mount and fasten the camera mount on the chassis-up using M3x10 flathead screws and M3 nuts from the Duckiebox.

Protip: make sure that the camera mount is: (a) geometrically centered on the chassis-up; (b) fastened as forward as it can go; (c) it is tightly fastened. We aim at having a standardized position for the camera and to minimize the wiggling during movement.

<figure id="camera_raspi_enssemble" figure-caption=" Raspberry Pi and camera with short cable">
     <img src="camera_raspi_enssemble.jpg" style='width: 30em'/>
</figure>

Note: If you only have a short camera cable, make sure that the cable is oriented in this direction (text on cable towards the CPU). Otherwise you will have to disassemble the whole thing later. On the long cable the writing is on the other side.

### Extending the intra-decks standoffs

In order to fit the battery, we will need to extend the Magician chassis standoffs with the provided nylon standoff spacers. Grab 4 of them, and secure them to one end of the long metal standoffs provided in the Magician chassis package.

Secure the extended standoff to the 4 corners of the chassis-bottom.  The nylon standoffs should smoothly screw in the metal ones. If you feel resistance, don’t force it or the nylon screw might break in the metal standoff. In that case, unscrew the nylon spacer and try again.

<figure id="standoff_extender" figure-caption="4 nylon M3x5 extended standoffs and 4 M3x6 metal screws from Magician chassis package">
     <img src="extender_screws.jpg" style='width: 25em'/>
</figure>

### Fasten the Battery with zip ties

Put the battery between the upper and lower decks of the chassis. It is strongly recommended to secure the battery from moving using zip ties.

<figure id="battery-zipties" figure-caption="Secure the battery to the chassis-top through the provided zipties. One can do the trick, two are better.">
     <img src="placeholder.jpg" style='width: 30em'/>
</figure>

Note: [](#fig:battery-zipties) can be taken as an example of how to arrange the long camera cable as well.

### Assemble chassis-bottom and chassis-up

Arrange the motor wires through the chassis-up, which will be connected to Stepper Motor HAT later.

<figure id="bottom_up_enssemble" figure-caption="The motor wires go through the center of chassis-up">
     <img src="bottom_up_enssemble.jpg" style='width: 25em'/>
</figure>

<figure id="screws_standoff" figure-caption="Side view of metal screws and the extended standoffs">
     <img src="screw_standoffs.jpg" style='width: 25em'/>
</figure>

Note: Use the provided metal screws from chassis package for fastening the chassis-up above the nylon standoffs instead of the provided M3 nylon screws.


### Place the DC Motor hat on top of the Raspberry Pi

Make sure the GPIO stacking header is carefully aligned with the underlying GPIO pins before applying pressure.

Note: In case with short camera cable, ensure that you doesn't break the cable while mounting the HAT on the Raspberry Pi. In case with long camera cable,


<figure id="GPIO_header" figure-caption="Assembled DC motor hat with short camera cable">
     <img src="GPIO_header.jpg" style='width: 30em'/>
</figure>

TODO: insert pic with long camera cable

### Connect the motor's wires to the terminal

We are using M1 and M2 terminals on the DC motor hat. The left (in robot frame) motor is connected to M1 and the right motor is connected to M2. If you have followed Part A correctly, the wiring order will look like as following pictures:

- Left Motor: Red
- Left Motor: Black
- Right Motor: Black
- Right Motor:Red

<!--
### Power supply for DC motor HAT

Attach the prepared power wires in previous section [](#power-cable-prep-c0) to the DC motor HAT power terminal block. Make sure you plug the black wire in the pin labeled with a minus: - and the red wire to the plus: + ([](#fig:power_terminal)). Plug the other end of the cable in the battery.

<div figure-id="fig:power_terminal" figure-caption="Insert the prepared power wire to DC motor HAT power pins">
     <img src="placeholder.jpg" style='width: 30em'/>
</div>
-->

### Connect the power cables

You are now ready to secure the prepared power wires in [](#power-cable-prep-c0) to the DC motor HAT power pins.

Connect the **battery** (not the Raspberry Pi) with the DC motor HAT by making sure you plug the black wire in the pin labeled with a minus: `-` and the red wire to the plus: `+` ([](#fig:final-result-power-c0)).

Fix all the cables on the Duckiebot so that it can run on the way without barrier.

<figure id="Stepper_cable" figure-caption="Insert the prepared power wire to DC motor HAT power pins.">
     <img src="Stepper_cable.jpg" style='width: 30em'/>
</figure>

Note: If you have a `DB17-Montreal-a` or `DB17-Chicago-a` release, neglect this step and follow the pertinent instructions in [](#assembling-duckiebot-c1) regarding the assembly of the PWM hat, its powering through the short angled USB cable, and the power transfer step using a M-M wire.

### Joypad

With each joypad ([](#fig:joypad)) comes a joypad dongle ([](#fig:joypad_dongle)). Don't lose it!

<figure id="joypad" figure-caption="All components in the Joypad package">
     <img src="joypad.jpg" style='width: 25em'/>
</figure>

Insert the joypad dongle into one of the USB port of the Raspberry Pi.

<figure id="joypad_dongle" figure-caption="The dongle on the Raspberry Pi">
     <img src="joypad_dongle.jpg" style='width: 25em'/>
</figure>

Insert 2 AA batteries on the back side of the joypad [](#fig:joypack_battery).

<figure id="joypack_battery" figure-caption="Joypad and 2 AA batteries">
     <img src="joypack_battery.jpg" style='width: 25em'/>
</figure>

## FAQ {#op-assembly-db17jwd-faq}

Q: If we have the bumpers, at what point should we add them?

A: You shouldn't have the bumpers at this point. The function of bumpers is to keep the LEDs in place, i.e., they belong to `DB17-l` configuration. These instructions cover the `DB17-jwd` configurations. You will find the bumper assembly instructions in [](#assembling-duckiebot-c1).

Q: Yeah but I still have the bumpers and am reading this page. So?

A: The bumpers can be added after the Duckiebot assembly is complete.

Q: I found it hard to mount the camera (the holes weren't lining up).

A: Sometimes in life you have to push a little to make things happen. (But don't push too much or things will break!)

Q: The long camera cable is a bit annoying - I folded it and shoved it in between two hats.

A: The shorter cable is even more annoying. We suggest wrapping the long camera cable between the chassis and the Raspberry Pi. With some strategic planning, you can use the zipties that keep the battery in place to hold the camera cable in place as well ([see figure below-to add]())

TODO: add pretty cable handling pic

Q: I found that the screwdriver that comes with the chassis kit is too fat to screw in the wires on the hat.

A: It is possible you got one of the fatter screwdrivers. You will need to figure it out yourself (or ask a TA for help).

Q: I need something to cut the end of the zip tie with.

A: Scissors typically work out for these kind of jobs (and no, they're not provided in a Fall 2017 Duckiebox).

<!--

Comment: Notes - if we have the bumpers, at what point should we add them? I think that the battery could actually be attached before the levels of the chassis are joined. I found it hard to mount the camera (the holes weren't lining up). the long camera cable is a bit annoying - I folded it and shoved it in between two hats. We should decide if PWM hat is part of this configuration, why not leave it for now and forget about the spliced cable for the class. I found that the screwdriver that comes with the chassis kit is too fat to screw in the wires on the hat. The picture of where to put the zip tie for the battery is not very clear. need something to cut the end of the zip tie with.

-->

<!--

Comment: In general I would recommend having diagonal pliers as well as a few mini screwdrivers at hand. Both can be obtained from a local dollar store for about 6$ total. The pliers / cutters are required either for making your own power cord or for cutting the zip ties after they've been attached to the chassis (because they are too long). The screwdrivers are required for tightening the screws on the hats after the cables have been plugged in because the chassis screwdriver is too wide for that.

-->
