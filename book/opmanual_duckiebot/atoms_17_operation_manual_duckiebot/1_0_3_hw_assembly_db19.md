# Assembling the Duckiebot `DB-19` {#assembling-duckiebot-db19 status=ready}

This page is for the `DB-19` configuration used in classes in 2018 and 2019.
This instructions are basically the same as the instructions for the DB18 (see here: [](#assembling-duckiebot-db18). However this version uses other dc motors with a built-in encoder. For older instructions from 2017 see the [`DB17` Duckiebot](https://docs.duckietown.org/DT17/) operation manual.

<div class='requirements' markdown="1">

Requires: Duckiebot `DB-19` parts. The acquisition process is explained in [](#duckiebot-configurations).

Requires: A microSD card with the Duckiebot image already on it. This procedure is explained [here](#setup-duckiebot).

Requires: 1.5 hours of assembly time.

Result: An assembled Duckiebot in configuration `DB-19`.

</div>

Note: Make sure you visit the [FAQ](#op-faq-db19) section at the bottom of this page. This may already answer your comments, questions or doubts.

The complete assembly process is divided into 9 subparts. Make sure to complete them in the following order:

- [Part 1: What is in the box?](#howto-bom-db19)
- [Part 2: Motors](#howto-motors-db19)
- [Part 3: Bottom chassis](#howto-bottom-chassis-db19)
- [Part 4: Computation unit](#howto-comp-unit-db19)
- [Part 5: Camera](#howto-camera-db19)
- [Part 6: Top chassis](#howto-top-chassis-db19)
- [Part 7: Back bumper](#howto-back-bumper-db19)
- [Part 8: Front bumper](#howto-front-bumper-db19)
- [Part 9: Battery and Duckie](#howto-battery-duckie-db19)

## What is in the box {#howto-bom-db19}

All the pieces in your Duckiebox are shown in [](#fig:duckiebot-components-db19). Note that the battery and camera calibration pattern are not shown in the picture.

Moreover, you might have slightly different components than those shown: for example, different USB power cables, slightly different sets of screws or a camera mount and backplate of different colors. Do not worry, these instructions can be followed anyway.

<div figure-id="fig:duckiebot-components-db19" figure-caption="Components in Duckiebot package (Duckiebox).">
     <img src="Whats-in-the-box-ks-db18-v1.png" style='width: 37em' />
</div>


Some of the components in your Duckiebox will not be used at this stage, e.g., the traffic signs and stands. Keep these aside, they will come in handy in other parts of the book.

Finally, you should have several spares, especially for the mechanical bits (nuts and screws). These are included just in case you drop a few and can't find them anymore (especially the tiny nylon ones like to hide in the cracks!).

## Preliminary Steps

### Unboxing
Unbox all of your components and lay them out on a flat surface. Ensure that you have well lit, uncluttered space to work on.

Although not necessary, you might find useful getting a small (M2.5) wrench to ease some of the passages.

### Do the bumper braces fit?
Take the rear bumper bracers and the back bumper. The back bumper will be mounted in the last steps as a press fit to the rear bumper bracers. Try to fit the bracers into the holes of the back bumper.

Some bumper bracers have a plastic protective film which is a residue of the manufacturing process. If you struggle in the press fitting, peel off the plastic cover from *one* side of the bracer. If this does not help, peel off the plastic cover from both sides.

### Plastic cover
Peel the plastic cover from all the chassis parts (except the bumper bracers) on both sides.

### Screws
Note that a few among all of your metal screws are special. They are "countersunk" screws. Keep these aside. They will be needed in [Part 7: Back bumper](#howto-back-bumper-db19).

<div figure-id="fig:countersunk_screw_db19" figure-caption="Countersunk screws have a tapered head.">
     <img src="countersunk_screw.png" style='width: 20em'/>
</div>

Every time you read M3x8 screw, a M3x10 will do the same trick. You can exchange them at will.

Do not exchange metal and nylon screws though. The latter are not electrically conductive and are passive protections to potential short circuits that can damage your Duckiebot beyond repair. This is especially true for - [Part 3: Bottom chassis](#howto-bottom-chassis-db19); make sure you use the correct nylon screws at that step.

Remember that instructions are your friend! At least at this stage, try to follow them precisely.

If regardless of this foreword you still choose to try and figure it out yourself, it's ok, but for the love of what is precious, *do not plug the battery in until you have performed a visual inspection* [here](#db19-assembly-visual-inspection).  


## Motors {#howto-motors-db19}

From the Duckiebox package take the following components:

- Bottom plate (1x)
- Blue dc motors (2x)
- Motor holders (4x)
- M3x30 screw (4x)
- M3 nuts (4x)

### Step 1
Pass two of the motor holders through the openings in the bottom plate of the chassis as shown
in [](#fig:db19-step_1a). If you have troubles fitting the holders, it is probably because you have not removed the protective plastic film. Note which holes you are using. You should be using the middle section ones.

<div figure-id="fig:db19-step_1a" figure-caption="How to mount the motor holders.">
     <img src="db19-step_1a.png" style='width: 25em'/>
</div>

Put one motor between the holders as shown in [](#fig:db19-step_1b).

<div figure-id="fig:db19-step_1b" figure-caption="How to mount the first DC-motor.">
  <img src="db19-step_1b.png" style='width: 25em'/>
</div>

Note: Orient the motors that their wires are pointing to the inside (i.e., towards the center of the plate).

Note: Use your screwdriver.

Use *2* M3x30 screws and *2* M3 nuts to secure the motor to the motor supports. Pass the screws through the holes from the outside inwards, then tighten the screws to secure the holders to the bottom plate of the chassis.

Note: You might find aligning the holes to be a little bit hard. It is easier to first get the top screw aligned and place the screw in, and then push in the side support piece in to place. If that still doesn't work, get the bottom screw and the side support in first, align it, and secure with nuts. Then try to push the top screw through. You might have to use the screw to expand the hole a bit.

### Step 2
Do the same for the second motor as well as shown in [](#fig:db19-step_2a) and [](#fig:db19-step_2b)

<div figure-id="fig:db19-step_2a" figure-caption="How to mount the second motor supports">
  <img src="db19-step_2a.png" style='width: 25em'/>
</div>

<div figure-id="fig:db19-step_2b" figure-caption="How to mount the second DC-motor">
  <img src="db19-step_2b.png" style='width: 25em'/>
</div>

Note: Tighten the motors firmly, as a wobbly motor might cause troubles later on.

### Step 3

Wire the cables of the two motors through the bottom plate of the chassis and make sure they are well separated ([](#fig:db19-step_3)).

<div figure-id="fig:db19-step_3" figure-caption="Cables through central hole">
   <img src="db19-step_3.png" style='width: 25em'/>
</div>


## Bottom chassis {#howto-bottom-chassis-db19}
The Duckiebot is driven by controlling the wheels attached to the DC motors.
Still, it requires a _passive_ support on the back. In this configuration an omni-directional wheel is
attached to the bottom plate of the chassis to provide such support.

From the Duckiebox package take the following components:
- Steel omni-directional wheel (1x)
- M3x25 metal spacers (2x)
- M3x10 metal screws (2x)
- M3x10 nylon screws (2x)

### Step 4
Assemble the omni-directional wheel as shown in figure [](#fig:db19-step_4).

<div figure-id="fig:db19-step_4" figure-caption="Assembly of the omni-directional wheel">
     <img src="db19-step_4.png" style='width: 25em'/>
</div>

### Step 5
Then mount it to the bottom plate of the Duckiebot as shown in figure [](#fig:db19-step_5).

<div figure-id="fig:db19-step_5" figure-caption="Mounting the omni-directional wheel">
     <img src="db19-step_5.png" style='width: 25em'/>
</div>

Note: Sometimes you might need a bit of force to get the screw and the standoffs fully connected.

### Step 6
From the Duckiebot kit take the following components:

- M3x10 screws (4x)
- M3x25 metal spacers (2x)
- M3x30 metal spacers (2x)
- 1x Rear bumper bracer

and mount the rear bracer to the bottom plate with the _25cm_ spacers ([](#fig:db19-step_6))!

<div figure-id="fig:db19-step_6" figure-caption="Mounting the rear bracer">
     <img src="db19-step_6.png" style='width: 25em'/>
</div>


### Step 7
Now use the M3x30 standoffs (longer ones) for the front of the Duckiebot ([](#fig:db19-step_7)).

<div figure-id="fig:db19-step_7" figure-caption="Mount front spacers">
     <img src="db19-step_7.png" style='width: 25em'/>
</div>

### Step 8
From the Duckiebot kit take the following components:

- Wheels (2x)

[](#fig:db19-step_8) shows how to mount the big yellow driving wheels to the bottom chassis.

<div figure-id="fig:db19-step_8" figure-caption="Mounting the driving wheels">
     <img src="db19-step_8.png" style='width: 25em'/>
</div>

Note: Mind that there is a particular orientation at which the wheels will fit. Don't force them too much!

[](#fig:howto-mount-wheels-milestone-db19) shows how the assembly should look like after mounting the wheels.

<div figure-id="fig:howto-mount-wheels-milestone-db19" figure-caption="The wheels mounted on the bottom plate assembly.">
   <img src="howto_mount_wheels_milestone_db18-1.jpg" style='width: 25em'/>
</div>


## Computation unit {#howto-comp-unit-db19}
From the Duckiebot kit take the following components:

- Raspberry Pi 3B+ (1x)
- Heat sink (1x) (or 2x if there are two of them)
- Camera cable (1x) (This should be in the same box as the camera)
- Micro SD card (1x)

Note: You probably have two heat sinks, make sure you use the bigger one for sure. The smaller one can be used as shown in the following pictures or apply it on the USB port you will later use for the external 32GB USB dongle.

Note: You will notice there is a camera cable already attached to the camera. We won't use this one since it is a bit short. You will find a longer camera cable inside the same box as the camera.

[](#fig:howto-mount-rpi-parts-db19) shows the components needed to complete the following steps.

<div figure-id="fig:howto-mount-rpi-parts-db19" figure-caption="The bigger heat sinks and the Raspberry Pi 3 B+.">
     <img src="howto_mount_rpi_parts.jpg" style='width: 25em'/>
</div>

### Step 9
Peel the cover from the bottom of the heat sink and place it on the Raspberry Pi microchip, as shown in [](#fig:db19-step_9). Make sure to put it on the

<div figure-id="fig:db19-step_9" figure-caption="How to mount the heat sink(s)">
     <img src="db19-step_9.png" style='width: 25em'/>
</div>

### Step 10
Insert the SD card in the slot as shown in figure [](#fig:db19-step_10).

Note: If the card is not flashed yet, do the initialization first: [](#setup-duckiebot), but only until the section [](#burn-sd-card), because you will need the fully assembled Duckiebot for the following steps of the Duckiebot Initialization.

<div figure-id="fig:db19-step_10" figure-caption="Insert SD card">
     <img src="db19-step_10.png" style='width: 25em'/>
</div>

### Step 11
Make sure that the visible metal connectors of the camera cable match the ones in the Raspberry Pi ([](#fig:db19-step_11a)), then plug in the cable and push down the black wings to fasten the connection, making sure it "clicks".

Note: Please be aware that different camera cables have the text on different sides and with different orientation, **do not** use it as a landmark.

<div figure-id="fig:db19-step_11a" figure-caption="Put the camera cable in the open plug">
     <img src="db19-step_11a.png" style='width: 25em'/>
</div>

Then close the camera plug once the cable is pushed in properly. You can check by pulling at the cable once you have closed the plug and you mustn't be able to tear it out at all.

<div figure-id="fig:db19-step_11b" figure-caption="Close camera cable plug">
     <img src="db19-step_11b.png" style='width: 25em'/>
</div>


### Step 12
From the Duckiebot kit take the following components:

- Duckiebot Hut (1x)
- Top plate of the chassis (1x)
- M2.5x12 nylon spacers (4x)
- M2.5x10 Nylon screws (4x)
- M2.5x4 Nylon spacers (4x)
- M2.5 nylon nuts (4x)
- USB to micro USB cables (2x)
- Set of three female to female(F/F) jumper wires (1x)

[](#fig:howto-assemble-hut-parts-db19) shows the components needed to complete this part of the tutorial.

<div figure-id="fig:howto-assemble-hut-parts-db19" figure-caption="The parts needed to assemble the Raspberry Pi and the Hut.">
     <img src="howto_assemble_hut_parts-1.jpg" style='width: 25em'/>
</div>

Note: In the newest version of the Hut, the pins are removed. This is to prevent them from being shorted. You will not need them so you can neglect them in the next few pictures.   

Note: It is more convenient to not separate the jumper cables, but leave them in two sets of three.

Place the spacers on the bottom part of the Duckiebot hut and secure them using the M2.5 nylon nuts, see figure [](#fig:db19-step_12)
<div figure-id="fig:db19-step_12" figure-caption="Mount spacers to Hut">
     <img src="db19-step_12.png" style='width: 25em'/>
</div>

### Step 13
Mount the pre-assembled Hut and Raspberry Pi to the top plate with nylon screws as shown in figure [](#fig:db19-step_13)

<div figure-id="fig:db19-step_13" figure-caption="Mount computation unit">
     <img src="db19-step_13.png" style='width: 25em'/>
</div>

Note: Make sure that the Raspberry Pi GPIO pins fit into the Hut connector.

### Step 14
Connect the F/F jumper wire to the pins on the Hut as shown in figure [](#fig:db19-step_14) and make sure the camera cable is put in the slit of the Hut

<div figure-id="fig:db19-step_14" figure-caption="Connect jumper wire">
     <img src="db19-step_14.png" style='width: 25em'/>
</div>

### Step 15
Finally, attach the two USB cables to the power plugs on the Hut and Raspberry Pi, as in [](#fig:db19-step_15).

<div figure-id="fig:db19-step_15" figure-caption="Plugging in the power cables">
     <img src="db19-step_15.png" style='width: 25em'/>
</div>

## Camera {#howto-camera-db19}
From the Duckiebox package take the following components:

- Raspberry Pi camera
- Camera mount (1x)
- M3x10 screws (3x)
- M3 nuts (3x)

### Step 16
Open the camera cable plug on the Raspberry Pi camera as shown in figure [](#fig:db19-step_16a).

<div figure-id="fig:db19-step_16a" figure-caption="Open camera cable plug">
     <img src="db19-step_16a.png" style='width: 25em'/>
</div>

Note: Make sure the cable is routed through the camera mount as well!

Close the plug once the cable is pushed in completely as in [](#fig:db19-step_16b)

<div figure-id="fig:db19-step_16b" figure-caption="Close camera plug">
     <img src="db19-step_16b.png" style='width: 25em'/>
</div>

Insert the M3x10 nylong screws from the front to screw the camera to its mount, see [](#fig:db19-step_16c)

<div figure-id="fig:db19-step_16c" figure-caption="Screw the camera to its mount">
     <img src="db19-step_16c.png" style='width: 25em'/>
</div>

Note: Make sure the camera is as parallel to its mount as possible, i.e. facing downwards at this very angle.

### Step 17
Screw the camera mount to the top plate with three M3x10 screws and metal M3 nuts, see [](#fig:db19-step_17).

<div figure-id="fig:db19-step_17" figure-caption="Screw the mount to the top plate">
     <img src="db19-step_17.png" style='width: 25em'/>
</div>

## Top chassis {#howto-top-chassis-db19}
From the Duckiebox package take the following components:
- Metal screws M3x10 (4x)
- Back bumper bracer (1x)


### Step 18
Put the top plate on the bottom part of the chassis as shown in figure [](#fig:db19-step_18) but do not screw it in yet. This is how it should look like in the end.

<div figure-id="fig:db19-step_18" figure-caption="Placing the top plate">
     <img src="db19-step_18.png" style='width: 25em'/>
</div>

### Step 19
Now open the slots for the cables of the DC motors on the Hut with a small screw driver and make sure you mount the cables in correctly, see figure [](#fig:db19-step_19). Probably you have to take the top and bottom part of the chassis apart again

<div figure-id="fig:db19-step_19" figure-caption="Connect the DC motor cables to the Hut">
     <img src="db19-step_19.png" style='width: 25em'/>
</div>


### Step 20
Place the second rear bumper bracer on top of the chassis spacers in the back of the chassis. Again, the holes should align ([](#fig:db19-step_20)). If you experience a slight misalignment, carefully pass an M3 screw through the top plate and the top bumper bracer first, then move the standoff until you can plug the screw in. At the end, put in the second M3 screw as well.

<div figure-id="fig:db19-step_20" figure-caption="Rear bumper bracer positioning">
     <img src="db19-step_20.png" style='width: 25em'/>
</div>

### Step 21
Fasten the top plate in the front using the other two M3x10 screws, as in([](#fig:db19-step_21).

<div figure-id="fig:db19-step_21" figure-caption="Fasten the top plate completely">
     <img src="db19-step_21.png" style='width: 25em'/>
</div>

### Step 22
You can use a zip tie to help with cable management ([](#fig:db19-step_22)). Tip: you can use the same zip tie to keep the camera cable down and the USB cables below from touching the wheels.

<div figure-id="fig:db19-step_22" figure-caption="Cable management">
     <img src="db19-step_22.png" style='width: 25em'/>
</div>

## Back bumper {#howto-back-bumper-db19}
From the Duckiebot kit take the following components:

- Back bumper (1x)
- M3x10 nylon screws (2x)
- M3x10 metal screws (4x)
- M3x10 countersunk screws (2x)
- M3 metal nut (4x)
- M3x25 spacers (2x)
- Back pattern plate (1x)
- Circle pattern sticker (1x)

Note: In the picture, nylon screws are used. However you probably used them to assemble the omni-directional wheel, then use the metal screws.

[](#fig:howto-mount-circlegrid-holder-parts-db19) shows the components needed to complete this upgrade.

<div figure-id="fig:howto-mount-circlegrid-holder-parts-db19" figure-caption="The parts needed to mount the circle grid holder.">
     <img src="howto_mount_circlegrid_holder_parts.png" style='width: 25em'/>
</div>

Note: You could have a back plate of a different color with respect to the picture, e.g., black or white. They are all functionally equivalent.

### Step 23
Mount the spacers using the *metal* screws, as in [](#fig:db19-step_23).

<div figure-id="fig:db19-step_23" figure-caption="How to mount the spacers">
     <img src="db19-step_23.png" style='width: 25em'/>
</div>

Note: Pay attention that the spacers should be in the same direction as the LEDs.


### Step 24
Fasten the back plate to the back bumper standoffs using the countersunk screws. The back plate is not symmetric, place it such that it is centered with respect to the back bumper as shown in picture [](#fig:db19-step_24).

<div figure-id="fig:db19-step_24" figure-caption="Mount the pattern plate">
     <img src="db19-step_24.png" style='width: 25em'/>
</div>

### Step 25
Peel off the sticker with the circle pattern and put it on the pattern plate mounted before, see picture [](#fig:db19-step_25).

<div figure-id="fig:db19-step_25" figure-caption="Pattern sticker">
     <img src="db19-step_25.png" style='width: 25em'/>
</div>

### Step 26
Connect a new jumper wire (F/F) to the connector on the back bumper, see picture [](#fig:db19-step_26).

<div figure-id="fig:db19-step_26" figure-caption="Jumper wire to back bumper">
     <img src="db19-step_26.png" style='width: 25em'/>
</div>

No matter what exact solution you choose concerning the cable management, the objective is to avoid any cables from touching the wheels in the end. One solution could be something like it is shown in figure [](#fig:howto-mount-front-bumper-db19).

<div figure-id="fig:howto-mount-front-bumper-db19" figure-caption="Cable management">
     <img src="howto_mount_front_bumper_2.png" style='width: 25em'/>
</div>

### Step 27
Screw the back bumper to the rest of the chassis with four metal screws and nuts, see picture [](#fig:db19-step_27).

<div figure-id="fig:db19-step_27" figure-caption="Mounting the back bumper">
     <img src="db19-step_27.png" style='width: 25em'/>
</div>

Note: Pay attention that the pins of the bumper bracers are properly put in the holes of the back bumper. It might be a little bit difficult to place it properly.

Note: Wire the cable mounted in step 26 through the top plate of the chassis as shown in the picture as well.


## Front bumper {#howto-front-bumper-db19}

From the Duckiebot kit take the following components:

- Front bumper (1x)
- M3x10 metal screws (2x)

### Step 28
At that point you should have arrived with two loose cables in the front. Make sure to connect both of them to the front bumper. The one from step 26 on the right hand side and the other on the left hand side, see picture [](#fig:db19-step_28).

<div figure-id="fig:db19-step_28" figure-caption="Wiring the front bumper">
     <img src="db19-step_28.png" style='width: 40em'/>
</div>

### Step 29
First, click in the front bumper without any screws. Maybe you have to bend the top and bottom plate of the chassis a little bit in order to make it fit, see pictures [](#fig:db19-step_29a) and [](#fig:db19-step_29b)

<div figure-id="fig:db19-step_29a" figure-caption="">
     <img src="db19-step_29a.png" style='width: 25em'/>
</div>

<div figure-id="fig:db19-step_29b" figure-caption="Clicking in the front bumper">
     <img src="db19-step_29b.png" style='width: 25em'/>
</div>

### Step 30
Now use the two M3x10 metal screws to finally fix the top plate of the chassis properly. This will now prevent the front bumper from falling out again. Probably you will have to move the standoffs a little bit in order to make the holes flush, see figure [](#fig:db19-step_30).

<div figure-id="fig:db19-step_30" figure-caption="Fixing the top plate">
     <img src="db19-step_30.png" style='width: 25em'/>
</div>


## Battery and Duckie {#howto-battery-duckie-db19}

From the Duckiebot kit take the following components:

- Battery (1x)
- Zip tie (1x)
- Duckie (N+1x)

### Visual inspection {#db19-assembly-visual-inspection}

Before plugging in the battery, make sure the Hut's GPIO pins are (a) not touching each other, (b) not touching any metal screws (in case you did not follow these instructions exactly) and (c) are free from any external object that might have gotten stuck there during the assembly process.

Note: If the GPIOs of the Hut get shorted (there is an electrically conductive connection between them) when you plug in the battery, you might damage the Raspberry Pi beyond repair!

### Step 31
Place the battery and fix it using a zip tie ([](#fig:db19-step_31)), then connect the USB cables.

Note: **Do not** unplug/replug the bumper wires when the power is on. You could break the bumpers!

<div figure-id="fig:db19-step_31" figure-caption="Fixing the top plate">
     <img src="db19-step_31.png" style='width: 25em'/>
</div>

There are different batteries concerning the outer dimensions, so the result could look like either of the following pictures [](#fig:howto-mount-battery-duckie-db19) or [](#fig:howto-assemble-finish-milestone-db19):

<div figure-id="fig:howto-mount-battery-duckie-db19" figure-caption="Solution with smaller battery">
     <img src="howto_mount_battery_duckie_1.jpg" style='width: 25em'/>
</div>

<div figure-id="fig:howto-assemble-finish-milestone-db19" figure-caption="The final Duckiebot assembly, with longer form factor battery.">
   <img src="howto_assemble_finish_milestone-2.jpg" style='width: 25em'/>
</div>

Now place a Duckie on top of your brand new Duckiebot, but be careful not to hurt the Duckie!


### Check the outcome

<div figure-id="fig:db19-step_32" figure-caption="The complete Duckiebot 19">
   <img src="db19-step_32.png" style='width: 25em'/>
</div>

Note: As a final check, verify that no cable is touching the wheels. You can use the provided zip ties to ensure that cables stay out of the way.

## FAQ {#op-faq-db19}

Q: I found it hard to mount the omni-directional wheel / the back bumper / the Raspberry Pi because the holes weren't lining up.

A: Sometimes in life you have to push a little to make things happen. (But don't push too much or things will break!)

Q: I have a different color of the back patter plate, do I get the wrong one?

A: The color of this part can be of any and all are functionally equivalent.

Q: My battery is different from the one shown in the pictures! Did I get the wrong box?

A: If there is a duckie in or on your box, you most probably got the right one. We support different battery models. All supported models are functionally equivalent, although the form factor varies.
