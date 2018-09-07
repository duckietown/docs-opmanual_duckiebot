# Assembling the Duckiebot (`DB18`)  {#assembling-duckiebot-db18 status=ready}

Point of contact: Gianmarco Bernasconi, Jacopo Tani

Once you have received the parts and soldered the necessary components, it is time to assemble them in a Duckiebot. Here, we provide the assembly instructions for the configuration `DB18`.

<div class='requirements' markdown="1">

Requires: Duckiebot `DB18` parts. The acquisition process is explained in [](#acquiring-parts-c0).

Requires: An SD card with the Duckiebot image already on it.

Requires: Time: about 45 minutes.

Results: An assembled Duckiebot in configuration `DB18`.

</div>

Note: The [FAQ](#op-assembly-db18-faq) section at the bottom of this page may already answer some of you comments, questions or doubts.

This section is comprised of 15 parts. Some parts build upon some of the previous parts, so make
sure to follow them in the following order.

- [Part 0: What is in the box?](#bom-db18)
- [Part 1: Motors](#howto-mount-motors-db18)
- [Part 2: Omni-directional wheel](#howto-mount-omniwheel-db18)
- [Part 3: Assemble Onmi-directional wheel and bottom chassis](#howto-assemble-chassis-bottom)
- [Part 4: Spacers to bottom plate](#howto-mount-spacers)
- [Part 5: Wheels](#howto-mount-wheels-db18)
- [Part 6: Preparing the Raspberry Pi](#howto-mount-rpi)
- [Part 7: Raspberry Pi and Hut](#howto-prepare-rpi-hut-assemble)
- [Part 8: Camera mount](#howto-mount-cameramount)
- [Part 9: Raspberry Pi and top plate](#howto-assemble-rpi-top)
- [Part 10: Camera](#howto-mount-camera)
- [Part 11: Chassis assembly](#howto-assemble-chassis)
- [Part 12: Circlegrid holder](#howto-mount-circlegrid-holder)
- [Part 13: Front bumper assembly](#howto-mount-front-bumper)
- [Part 14: Back bumper assembly](#howto-mount-back-bumper)
- [Part 15: Battery and Duckie](#howto-mount-battery-duckie)

## What is in the box {#bom-db18}
Explain what the pieces are

All the pieces in your Duckiebox are shown in [](#fig:duckiebot-components-db18). Note that you might have a different battery than in the picture, or different USB to micro USB cables, but the instructions can be followed anyway.
<div figure-id="fig:duckiebot-components-db18" figure-caption="Components in Duckiebot package.">
     <img src="duckiebot_components_db18.jpg" style='width: 30em'/>
</div>

If you have got extra pieces, such as an extra heatsink or screws and nuts, just keep them apart, they could turn out to be useful in the future!


### Preliminary Steps

#### Step A

Unbox all of your components, and put aside the black battery holder from the Magician Chassis aswell as the two wheel encoders, they are not used.

#### Step B

Take the rear bumper bracers and the back bumper. The back bumper will be mounted in the last steps as a press fit to the rear bumper bracers. Try to fit the bracers into the holes of the back bumper, if you struggle peel off the plastic cover from *one* side. If this does not help, peel off the plastic cover from both sided.
Peel the plastic cover from all the chassis parts (except the bumper bracers) on both sides.

## Motors {#howto-mount-motors-db18}

From the Duckiebox package take the following components:

- Chassis-bottom (1x)
- DC Motors (2x)
- Motor holders (4x)
- M3x30 screw (4x)
- M3 nuts (4x)

[](#fig:howto-mount-motors-parts-db18) shows the components needed to complete this part of the tutorial.

<div figure-id="fig:howto-mount-motors-parts-db18" figure-caption="Components needed to mount the motors.">
     <img src="howto_mount_motors_parts.jpg" style='width: 30em'/>
</div>

### Step-by-step guide

#### Step 1

Pass two of the motor holders through the openings in the bottom plate of the chassis as shown
in [](#fig:howto-mount-motors-1).

<div figure-id="fig:howto-mount-motors-1" figure-caption="How to mount the motor holders.">
     <img src="howto-mount-motors-1.jpg" style='width: 30em'/>
</div>


#### Step 2

Put one motor between the holders as shown in [](#fig:howto-mount-motors-2).

<div figure-id="fig:howto-mount-motors-2" figure-caption="The sketch of how to mount a motor.">
  <img src="howto-mount-motors-2.jpg" style='width: 30em'/>
</div>

Note: Orient the motors so that their wires are inwards (i.e., towards the center of the plate).


#### Step 3

Use *2* M3x30 screws and *2* M3 nuts to secure the motor to the motor holders. Pass the screws through the holes from the outside inwards, then  tighten the screws
to secure the holders to the bottom plate of the chassis as shown in [](#fig:howto-mount-motors-3).

<div figure-id="fig:howto-mount-motors-3" figure-caption="The sketch of how to secure a motor to the bottom plate.">
   <img src="howto-mount-motors-3.jpg" style='width: 30em'/>
</div>

#### Step 4

Repeat for the opposite side and check that the outcome is the same as in [](#fig:howto-mount-motors-milestone)

<div figure-id="fig:howto-mount-motors-milestone-db18" figure-caption="The motors are attached to the bottom plate of the chassis.">
   <img src="howto-mount-motors-milestone-db18.jpg" style='width: 30em'/>
</div>

#### Step 5

Tilt your bottom plate and pass the cables of the motors through the central hole. Keep the cables coming from each motor separated ([](#fig:howto-mount-motors-milestone)).

<div figure-id="fig:howto-mount-motors-4" figure-caption="Cables through central hole.">
   <img src="howto-mount-motors-4.jpg" style='width: 30em'/>
</div>



## Omni-directional wheel {#howto-mount-omniwheel-db18}

The Duckiebot is driven by controlling the wheels attached to the DC motors.
Still, it requires a _passive_ support on the back. In this configuration an omni-directional wheel is
attached to the bottom plate of the chassis to provide such support.

From the Duckiebox package take the following components:

- Steel omni-directional wheel (1x)
- M3x25 metal spacers (2x)
- M3x10 screws (2x)


[](#fig:howto-mount-omniwheel-parts-db18) shows the components needed to complete this part of the tutorial.

<div figure-id="fig:howto-mount-omniwheel-parts-db18" figure-caption="The components for the omni-directional wheel assembly.">
     <img src="howto_mount_omniwheel_parts-db18.jpg" style='width: 30em'/>
</div>


### Step-by-step guide

#### Step 1

Secure the M3x25 spacers to the omni-directional wheel using *2* M3x10 screws as shown in [](#fig:howto-mount-omniwheel).

<div figure-id="fig:howto-mount-omniwheel" figure-caption="Parts needed to build the omni-directional wheel.">
     <img src="howto_mount_omniwheel-1.jpg" style='width: 30em'/>
</div>


## Assemble omni-directional wheel and bottom chassis {#howto-assemble-chassis-bottom}

From the previously prepared pieces take the following components:

- Assembled bottom chassis (1x)
- Assembled omni-directional wheel (1x)


From the Duckiebot kit take the following components:

- M3x10 screws (4x)
[](#fig:howto-assemble-omniwheel-parts) shows the components needed to complete this part of the tutorial.

<div figure-id="fig:howto-assemble-omniwheel-parts" figure-caption="How to assemble the omni-directional wheel.">
     <img src="howto_assemble_omniwheel.jpg" style='width: 30em'/>
</div>

### Step-by-step guide

#### Step 1
Pass the screws through the designated holes from the top, as shown in [](#fig:howto-assemble-omniwheel-1)



<div figure-id="fig:howto-assemble-omniwheel-1" figure-caption="Screws through the holes in the top plate.">
     <img src="howto_assemble_omniwheel_1.jpg" style='width: 30em'/>
</div>


#### Step 2

In the bottom part, secure the omni-directional wheel using the screws.


### Check the outcome

[](#fig:howto-mount-standoffs-milestone) shows how the omni-directional wheel should be attached to the plate.

<div figure-id="fig:howto-assembly-omniwheel-milestone" figure-caption="The omniwheel attached to the plate.">
   <img src="howto_assemble_omniwheel_milestone.jpg" style='width: 30em'/>
</div>




## Spacers {#howto-mount-spacers}

From the previously prepared pieces take the following components:

- Assembled bottom chassis

From the Duckiebot kit take the following components:

- M3x6 screws (2x)
- M3x10 screws (2x)
- M3x25 metal spacers (2x)
- M3x30 metal spacers (2x)
- 1x Rear bumper bracer

[](#fig:howto-mount-spacers-parts) shows the components needed to complete this part of the tutorial.

<div figure-id="fig:howto-mount-spacers-parts" figure-caption="The parts needed to fix the spacers to the bottom plate.">
     <img src="howto_mount_spacers_parts.png" style='width: 30em'/>
</div>


### Step-by-step guide

#### Step 1

Insert two screws in the designated holes and place the rear bumper on them, they should align as in [](#fig:howto-mount-spacers-1). A litte forcing might be necessary.
<div figure-id="fig:howto-mount-spacers-1" figure-caption="Place the screws and the rear bumper bracer.">
     <img src="howto_mount_spacers_1.png" style='width: 30em'/>
</div>

#### Step 2

Fasten the rear bumper using the M3x25 spacers (short ones), as in  [](#fig:howto-mount-spacers-2).

<div figure-id="fig:howto-mount-spacers-2" figure-caption="Fasten using the spacers.">
     <img src="howto_mount_spacers_2.png" style='width: 30em'/>
</div>

#### Step 3

Insert screws and M3x30 (long ones) spacers on the fron of the Duckiebot, in the position indicated in [](#fig:howto-mount-spacers-3).

<div figure-id="fig:howto-mount-spacers-3" figure-caption="Mount front spacers.">
     <img src="howto_mount_spacers_3.png" style='width: 30em'/>
</div>

## Wheels {#howto-mount-wheels-db18}

From the previously prepared pieces take the following components:

- Assembled bottom chassis

From the Duckiebot kit take the following components:

- Wheels (2x)

[](#fig:howto-mount-wheels-parts-db18) shows the components needed to complete this part of the tutorial.

<div figure-id="fig:howto-mount-wheels-parts-db18" figure-caption="The wheels and the current bottom assembly.">
     <img src="howto_mount_wheels_parts_db18.png" style='width: 30em'/>
</div>


### Step-by-step guide

#### Step 1

Insert one wheel on each motor, as in [](#fig:howto-mount-wheels-1)

<div figure-id="fig:howto-mount-wheels-1" figure-caption="How to mount the wheels.">
     <img src="howto_mount_wheels_1.png" style='width: 30em'/>
</div>

### Check the outcome

[](#fig:howto-mount-wheels-milestone-db18) shows how the assembly should look like after mounting the wheels.

<div figure-id="fig:howto-mount-wheels-milestone-db18" figure-caption="The wheels mounted on the bottom plate assembly.">
   <img src="howto_mount_wheels_milestone_db18.png" style='width: 30em'/>
</div>

## Preparing the Raspberry Pi {#howto-mount-rpi}

From the Duckiebot kit take the following components:

- Raspberry Pi 3B+ (1x)
- Heat sink (1x)
- Camera cable (1x)
- Micro SD card (1x)

[](#fig:howto-mount-rpi-parts) shows the components needed to complete this part of the tutorial.

<div figure-id="fig:howto-mount-rpi-parts" figure-caption="The heat sinks and the Raspberry Pi 3B+.">
     <img src="howto_mount_rpi_parts.jpg" style='width: 30em'/>
</div>

### Step-by-step guide

#### Step 1

Peel the cover from the bottom of the heat sink and place it on the Raspberry Pi microchip, as shown in [](#fig:howto-mount-rpi-1)
TODO: check if one or two

<div figure-id="fig:howto-mount-rpi-1" figure-caption="How to mount the heat sink.">
     <img src="howto_mount_rpi_1.jpg" style='width: 30em'/>
</div>

#### Step 2

Locate the camera cable plug (see [](#fig:howto-mount-rpi-2) for reference) in the Raspberry Pi and lift the black "wings" to open it.

<div figure-id="fig:howto-mount-rpi-2" figure-caption="Open the camera cable plug.">
     <img src="howto_mount_rpi_2.jpg" style='width: 30em'/>
</div>

#### Step 3

Make sure that the connectors of the camera cable match the ones in the Raspberry Pi ([](#fig:howto-mount-rpi-3)), then plug in the cable and push down the black wings to fasten the connection, making sure it "clicks".

Note: Please be aware that different camera
cables have the text on different sides and with different orientation, **do not** use it as a landmark.
<div figure-id="fig:howto-mount-rpi-3" figure-caption="Open the camera cable plug.">
     <img src="howto_mount_rpi_3.jpg" style='width: 30em'/>
</div>

#### Step 4

Locate the SD card holder on the Raspberry Pi (on the bottom). Then insert the micro SD card as in [](#fig:howto-mount-rpi-4)
<div figure-id="fig:howto-mount-rpi-4" figure-caption="Open the camera cable plug.">
     <img src="howto_mount_rpi_4.png" style='width: 30em'/>
</div>


### Check the outcome

[](#fig:howto-mount-rpi-milestone-db18) shows how the Raspberry Pi should look like now.

<div figure-id="fig:howto-mount-rpi-milestone-db18" figure-caption="The heat sinks installed on the Raspberry Pi 3.">
   <img src="howto_mount_rpi_milestone_db18.jpg" style='width: 30em'/>
</div>


## Raspberry Pi and Hut {#howto-prepare-rpi-hut-assemble}

From the previously prepared pieces take the following components:

- Assembled Raspberry Pi

From the Duckiebot kit take the following components:

- Duckietown Hut (1x)
- M2.5x12 nylon spacers (4x)
- M2.5 nylon nuts (4x)
- USB to micro USB cables (2x)
- Set of three female to female(F/F) jumper wires (1x)

[](#fig:howto-assemble-hut-parts) shows the components needed to complete this part of the tutorial.

Note: It is cleaner if you do not separate each cable, but leave them in two sets of three.

<div figure-id="fig:howto-assemble-hut-parts" figure-caption="The parts needed to assemble the Raspberry Pi and the Hut.">
     <img src="howto_assemble_hut_parts.jpg" style='width: 30em'/>
</div>


### Step-by-step guide

#### Step 1

Place the spacers on the bottom part of the Duckietown hut, as in [](#fig:howto-assemble-hut-1)

<div figure-id="fig:howto-assemble-hut-1" figure-caption="Position of the nylon spacers.">
     <img src="howto_assemble_hut_1.png" style='width: 30em'/>
</div>

#### Step 2

From the top of the Duckietown Hut, secure using the M2.5 nylon nuts.

#### Step 3

Place the Raspberry Pi as in [](#fig:howto-assemble-hut-2), passing the camera cable through the slit in the Duckietown Hut. Them plug in the Duckietown Hut, by making sure that the Raspberry Pi pins fit into the Hut connector. Plug in the USB cables in the two micro USB ports.

<div figure-id="fig:howto-assemble-hut-2" figure-caption="Position of the nylon spacers.">
     <img src="howto_assemble_hut_2.jpg" style='width: 30em'/>
</div>

#### Step 4

Take one of the two sets of F/F jumper wires, and attach it to the Duckietown Hut as shown in [](#fig:howto-assemble-hut-3). It is important to use the pins named: "5Vusb, ADR LED and GND", do *not* use the pin labelled "5Vraspi". Take note of which color have you connected to each pin.

<div figure-id="fig:howto-assemble-hut-3" figure-caption="Position of the nylon spacers.">
     <img src="howto_assemble_hut_3.jpg" style='width: 30em'/>
</div>


### Check the outcome

[](#fig:howto-assemble-rpi3-hut-milestone) shows how the Raspberry Pi 3 should look like at this point.
TODO: picture with F/F jumper cables and usb cables aswell

<div figure-id="fig:howto-assemble-rpi3-hut-milestone" figure-caption="The Raspberry Pi assembly.">
   <img src="howto_assemble_rpi3_hut_milestone.jpg" style='width: 30em'/>
</div>



## Camera mount {#howto-mount-cameramount}

From the Duckiebox package take the following components:

- Top plate (1x)
- Camera mount (1x)
- M3x10 screws (3x)
- M3 nuts (3x)

[](#fig:howto-mount-camera-mount-parts-db18) shows the components needed to complete this part of the tutorial.

<div figure-id="fig:howto-mount-camera-mount-parts-db18" figure-caption="The parts needed to secure the camera mount to the top plate.">
     <img src="howto_mount_camera_mount_parts.png" style='width: 30em'/>
</div>

### Step-by-step guide

#### Step 1

Place the camera mount over the top place as in [](#fig:howto-mount-camera-mount-1)

<div figure-id="fig:howto-mount-camera-mount-1" figure-caption="How to place the camera mount.">
     <img src="howto_mount_camera_mount_1.jpg" style='width: 30em'/>
</div>

#### Step 2

Insert the M3x10 screws from the opposite side of the camera mount (see [](#fig:howto-mount-camera-mount-2))

<div figure-id="fig:howto-mount-camera-mount-2" figure-caption="How to place the camera mount.">
     <img src="howto_mount_camera_mount_2.jpg" style='width: 30em'/>
</div>

#### Step 3

Fasten using M3 nuts ([](#fig:howto-mount-camera-mount-3))).

<div figure-id="fig:howto-mount-camera-mount-3" figure-caption="How to fasten the camera mount.">
     <img src="howto_mount_camera_mount_3.jpg" style='width: 30em'/>
</div>

## Raspberry Pi and top plate {#howto-assemble-rpi-top}

From the previously prepared pieces take the following components:

- Assembled chassis top plate
- Assembled Raspberry Pi

From the Duckiebot kit take the following components:

- M2.5x10 Nylon screws (4x)
- M2.5x10 Nylon spacers (4x)

[](#fig:howto-assemble-rpi-top-parts) shows the components needed to complete this part of the tutorial.

<div figure-id="fig:howto-assemble-rpi-top-parts" figure-caption="The parts needed to assemble the top plate and the Raspberry Pi.">
     <img src="howto_assemble_rpi_top_parts.png" style='width: 30em'/>
</div>

### Step by step instructions

#### Step 1

Place the M2.5 spacers on the bottom side of the Raspberry Pi, as in [](#fig:howto-assemble-rpi-top-1).

<div figure-id="fig:howto-assemble-rpi-top-1" figure-caption="Place the spacers on the Raspberry Pi.">
     <img src="howto_assemble_rpi_top_1.png" style='width: 30em'/>
</div>

#### Step 2

Place the Raspberry Pi under the top plate, positioned according to [](#fig:howto-assemble-rpi-top-2). Fasten it using the M2.5x10 nylon screws.

<div figure-id="fig:howto-assemble-rpi-top-2" figure-caption="Fasten the Raspberry Pi to the top plate.">
     <img src="howto_assemble_rpi_top_2.png" style='width: 30em'/>
</div>



## Camera {#howto-mount-camera}

From the previously prepared pieces take the following components:

- Top chassis assembly

From the Duckiebot kit take the following components:

- Camera (1x)
- M2x10 screws (4x)
- M2x10 nuts (nylon or metal) (4x)

[](#fig:howto-mount-camera-parts-db18) shows the components needed to complete this part of the tutorial.

<div figure-id="fig:howto-mount-camera-parts-db18" figure-caption="The parts needed to add the camera to the top plate.">
     <img src="howto_mount_camera_parts.jpg" style='width: 30em'/>
</div>

### Step-by-step guide

#### Step 1

Pass the camera cable through the top plate as in the picture, pay attention to the connector orientation, it must be the same as in [](#fig:howto-mount-camera-1).

<div figure-id="fig:howto-mount-camera-1" figure-caption="The orientation of the camera cable.">
     <img src="howto_mount_camera_1.jpg" style='width: 30em'/>
</div>

#### Step 2

Pass the camera cable through the camera mount ([](#fig:howto-mount-camera-2)).

<div figure-id="fig:howto-mount-camera-2" figure-caption="Cable management for camera cable.">
     <img src="howto_mount_camera_2.jpg" style='width: 30em'/>
</div>

#### Step 3

Pull the black wings of the camera port on the camera and plug in the camera cable. Pay attention that the pins on the camera cable match the pins on the camera. Close the connector by pushing the black wings making sure it “clicks”.
([](#fig:howto-mount-camera-3)).

<div figure-id="fig:howto-mount-camera-3" figure-caption="Camera connection.">
     <img src="howto_mount_camera_3.jpg" style='width: 30em'/>
</div>

Note: Please be aware that different camera
cables have the text on different sides and with different orientation, **do not** use it as a landmark.

#### Step 4

Place the camera on the camera mount, and insert the M2x10 screws from the front to the back, as in [](#fig:howto-mount-camera-4).
<div figure-id="fig:howto-mount-camera-4" figure-caption="How to fix the camera.">
     <img src="howto_mount_camera_4.jpg" style='width: 30em'/>
</div>

#### Step 5

Secure the camera from the back using the M2 nuts. ([](#fig:howto-mount-camera-5)).
<div figure-id="fig:howto-mount-camera-5" figure-caption="How to secure the camera.">
     <img src="howto_mount_camera_5.jpg" style='width: 30em'/>
</div>


## Chassis assembly {#howto-assemble-chassis}

From the previously prepared pieces take the following components:

- Assembled top chassis (1x)
- Assembled bottom chassis (1x)

From the Duckiebot kit take the following components:

- M3x10 screws (2x)
- Back bumper bracer (1x)

[](#fig:howto-assemble-chassis-parts) shows the components needed to complete this part of the tutorial.

<div figure-id="fig:howto-assemble-chassis-parts" figure-caption="The parts needed to add the battery to the Duckiebot.">
     <img src="howto_assemble_chassis_parts.png" style='width: 30em'/>
</div>

### Step-by-step guide

#### Step 1

Place the top chassis assembly over the bottom chassis assembly as in [](#fig:howto-assemble-chassis-1)

<div figure-id="fig:howto-assemble-chassis-1" figure-caption="The parts needed to add the battery to the Duckiebot.">
     <img src="howto_assemble_chassis_1.jpg" style='width: 30em'/>
</div>

#### Step 2

Note: Confusiong coming soon, aling your chackras and read the procedure through before starting.

For each motor, the cable that is soldered closer to the floor (bottom), goes on the *external* connection. The cable soldered on the motor on the plate side goes on the *internal* connection.

Insert the *right*-motor cable on the *right* side of the Hut connector ([](#fig:howto-assemble-chassis-2) perspective), and the *left*-motor cable on the *left* side of the Hut connector.

<div figure-id="fig:howto-assemble-chassis-2" figure-caption="Example for the connection.">
     <img src="howto_assemble_chassis_2.jpg" style='width: 30em'/>
</div>



For reference, look at the example in [](#fig:howto-assemble-chassis-3), where the *left* motor (driving perspective) has the *red* wire soldered on the bottom, so it goes on the *external left* connector. Similarly, the *right* motor (driving perspective) has the *black* wire soldered on the bottom, so it goes on the *external right* connector.

TODO: help for explicative image
<div figure-id="fig:howto-assemble-chassis-3" figure-caption="Explicative image.">
     <img src="placeholder.jpg" style='width: 30em'/>
</div>


#### Step 3

Flip the chassis top plate assembly, it should look like in [](#fig:howto-assemble-chassis-4).

<div figure-id="fig:howto-assemble-chassis-4" figure-caption="How should the chassis look like.">
     <img src="howto_assemble_chassis_4.jpg" style='width: 30em'/>
</div>

#### Step 4

Place the second rear bumper bracer on top of the chassis spacers in the back of the chassis, again the holes should align ([](#fig:howto-assemble-chassis-5)).

<div figure-id="fig:howto-assemble-chassis-5" figure-caption="Rear bumper bracer positioning.">
     <img src="howto_assemble_chassis_5.jpg" style='width: 30em'/>
</div>

#### Step 5

Fasten the top plate using the M3x10 screws, as in ([](#fig:howto-assemble-chassis-6)).

<div figure-id="fig:howto-assemble-chassis-6" figure-caption="How to fasten the chassis.">
     <img src="howto_assemble_chassis_6.jpg" style='width: 30em'/>
</div>

#### Step 6

Use a zip tie to help with cable management ([](#fig:howto-assemble-chassis-7)).

<div figure-id="fig:howto-assemble-chassis-7" figure-caption="Cable management.">
     <img src="howto_assemble_chassis_7.jpg" style='width: 30em'/>
</div>

## Circle grid holder {#howto-mount-circlegrid-holder}

From the Duckiebot kit take the following components:

- Back bumper (1x)
- M3x10 nylon screws (2x)
- M3x10 countersunk screws (2x)
- M3x25 spacers (2x)
- Back plate (1x)
- Circle pattern sticker (1x)

[](#fig:howto-mount-circlegrid-holder-parts) shows the components needed to complete this upgrade.

<div figure-id="fig:howto-mount-circlegrid-holder-parts" figure-caption="The parts needed to mount the circle grid holder.">
     <img src="howto_mount_circlegrid_holder_parts.png" style='width: 30em'/>
</div>

### Step-by-step guide

#### Step 1

Mount the spacers using the nylon screws, as in [](#fig:howto-mount-circlegrid-holder-1).

<div figure-id="fig:howto-mount-circlegrid-holder-1" figure-caption="How to mount the spacers.">
     <img src="howto_mount_circlegrid_holder_1.png" style='width: 30em'/>
</div>

Note: pay attention that the spacers should be in the same direction as the LEDs.

#### Step 2

Take the back plate and place the countersunk screws on the side where the holes are tapered, as shown in [](#fig:howto-mount-circlegrid-holder-2).

<div figure-id="fig:howto-mount-circlegrid-holder-2" figure-caption="How to mount the spacers.">
     <img src="howto_mount_circlegrid_holder_2.png" style='width: 30em'/>
</div>


#### Step 3

Fasten the back plate to the back bumper standoffs, like shown in the picture. The back plate is not symmetric, place it such that it is centered with respect to the back bumper. Then apply the circle pattern sticker to the back plate ([](#fig:howto-mount-circlegrid-holder-3)).

<div figure-id="fig:howto-mount-circlegrid-holder-3" figure-caption="How to the sticker.">
     <img src="howto_mount_circlegrid_holder_3.png" style='width: 30em'/>
</div>



## Front bumper assembly {#howto-mount-front-bumper}

From the previously prepared pieces take the following components:

- Assembled chassis (1x)

From the Duckiebot kit take the following components:

- Front bumper (1x)
- Set of three female to female jumper wires (1x)
- M3x6 Screws (2x)

[](#fig:howto-mount-front-bumper-parts) shows the components needed to complete this upgrade.

Note: It is cleaner if you do not separate each cable, but leave them in two sets of three.

<div figure-id="fig:howto-mount-front-bumper-parts" figure-caption="The parts needed to mount the front bumper.">
     <img src="howto_mount_front_bumper_parts.jpg" style='width: 30em'/>
</div>

### Step-by-step guide

#### Step 1
Place the front bumper in front of the chassis. Then locate the pins on the front bumper, you should have taken note of the connection you have done on the Duckietown Hut ([](#fig:howto-mount-front-bumper-1)).

Note: Pay attention to place the front bumper oriented as in the picture.

<div figure-id="fig:howto-mount-front-bumper-1" figure-caption="The parts needed to mount the front bumper.">
     <img src="howto_mount_front_bumper_1.jpg" style='width: 30em'/>
</div>

#### Step 2

Pass the electric F/F cables through the top chassis (cable management). Avoid anything touching the wheels ([](#fig:howto-mount-front-bumper-2)).

<div figure-id="fig:howto-mount-front-bumper-2" figure-caption="Cable management.">
     <img src="howto_mount_front_bumper_2.png" style='width: 30em'/>
</div>

Note: The ziptie in the picture is not mandatory but can be used to ease cable management

#### Step 3

Now we need to connect the two set of wires to the front bumper.
The set of cables coming from the Raspberry Pi must be connected on the right ([](#fig:howto-mount-front-bumper-3) perspective, namely the green blue and yellow wires in the picture), while the connector on the left (red white and black wires in the picture) will go to the back bumper. Again take note of which color you used for which pin in the left connector.

<div figure-id="fig:howto-mount-front-bumper-3" figure-caption="Front bumper connections.">
     <img src="howto_mount_front_bumper_3.jpg" style='width: 30em'/>
</div>

#### Step 5

Place the cabled front bumper in front of the chassis, with your fingers slightly lift the top plate and insert the front bumper. It is a form fit, the connector on the lower part must go in the circular hole (see [](#fig:howto-mount-front-bumper-5)), while the two top connectors will lock in just behind the camera mount (refer to [](#fig:howto-mount-front-bumper-6)).

<div figure-id="fig:howto-mount-front-bumper-5" figure-caption="Front bumper bottom part placement.">
     <img src="howto_mount_front_bumper_5.jpg" style='width: 30em'/>
</div>

<div figure-id="fig:howto-mount-front-bumper-6" figure-caption="Front bumper upper part placement..">
     <img src="howto_mount_front_bumper_6.jpg" style='width: 30em'/>
</div>

#### Part 6

Insert and fasten the screws trough the top plate in the front chassis spacers.


## Back bumper assembly {#howto-mount-back-bumper}

From the previously prepared pieces take the following components:

- Assembled chassis (1x)
- Back bumper assembly (1x)

From the Duckiebot kit take the following components:

- M3x10 screws (1x)

[](#fig:howto-mount-back-bumper-parts) shows the components needed to complete this upgrade.

<div figure-id="fig:howto-mount-back-bumper-parts" figure-caption="The parts needed to mount the back bumper.">
     <img src="howto_mount_back_bumper_parts.png" style='width: 30em'/>
</div>

### Step-by-step instructions

#### Step 1

Place the back bumper as shown in [](#fig:howto-mount-back-bumper-1), then connect the wires to the back bumper according to the connection done in the front bumper.

<div figure-id="fig:howto-mount-back-bumper-1" figure-caption="How to place the back bumper assembly.">
     <img src="howto_mount_back_bumper_1.png" style='width: 30em'/>
</div>

#### Step 2

The back bumper is press fit to the bumper bracers, place it as in [](#fig:howto-mount-back-bumper-3). You will have to apply little pressure, and it should fit tightly.

<div figure-id="fig:howto-mount-back-bumper-3" figure-caption="How to fix the back bumper.">
     <img src="howto_mount_back_bumper_3.png" style='width: 30em'/>
</div>

### Check the outcome

[](#fig:howto-assemble-back-bumper-milestone) shows how the current assebly should look like at this point.

<div figure-id="fig:howto-assemble-back-bumper-milestone" figure-caption="The Duckiebot assembly.">
   <img src="howto_mount_back_bumper_milestone.png" style='width: 30em'/>
</div>

## Battery and Duckie {#howto-mount-battery-duckie}

From the previously prepared pieces take the following components:

- Assembled chassis (1x)

From the Duckiebot kit take the following components:

- Battery (1x)
- Zip tie (1x)
- Duckie (N+1x)

[](#fig:howto-mount-battery-duckie-parts) shows the components needed to complete this tutorial.

<div figure-id="fig:howto-mount-battery-duckie-parts" figure-caption="The parts needed to mount the battery.">
     <img src="howto_mount_battery_duckie_parts.jpg" style='width: 30em'/>
</div>

### Step-by-step instructions

#### Step 1

Place the battery and fix it using a zip tie, then connect the USB cables, as in [](#fig:howto-mount-battery-duckie-1).

Note: **Do not** unplug/replug the bumper wires when the power is on. You could break the bumpers!

<div figure-id="fig:howto-mount-battery-duckie-1" figure-caption="The parts needed to mount the battery.">
     <img src="howto_mount_battery_duckie_1.jpg" style='width: 30em'/>
</div>

#### Step 2

Place the Duckie on top of your brand new Duckiebot.

Note: Be careful not to hurt the Duckie.

### Check the outcome

[](#fig:howto-assemble-finish-milestone) shows how the final assembly should look like. Congratulations!

<div figure-id="fig:howto-assemble-finish-milestone" figure-caption="The final Duckiebot assembly.">
   <img src="howto_assemble_finish_milestone.jpg" style='width: 30em'/>
</div>

## FAQ {#op-assembly-db18-faq}

Q: I found it hard to mount the omni-directional wheel (the holes weren't lining up).

A: Sometimes in life you have to push a little to make things happen. (But don't push too much or things will break!)
