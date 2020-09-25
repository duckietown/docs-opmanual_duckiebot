# Assembling the `DB-beta` Duckiebot {#assembling-duckiebot-db-beta status=ready}

This page is for the `DB-beta` configuration used in the AMoD class at ETH Zurich in 2020. The hardware used in this Duckiebot model is very similar to the hardware of the `DB19`. The major difference between these models is the replacement of the Raspberry Pi 3 by a Jetson Nano Nvidia board which lead to other minor hardware changes like a new camera, a cooling fan, layout, etc.

<div class='requirements' markdown="1">

Requires: Be enrolled in ETHZ’s AMoD 2020 course and have picked up your Duckiebox from the Autolab.

Requires: Time: about 1-1.5 hours (45 minutes for an experienced Duckiebot builder).

Results: An assembled Duckiebot in configuration `DB-beta`.

</div>

<!-- Note: The [FAQ](#op-faq-db19) section at the bottom of this page may already answer some of you comments, questions or doubts. -->


There are ?? parts in this procedure. Most parts build upon previous steps, so make
sure to follow them in the following order:

<!-- - [Part 1: What is in the box?](#howto-bom-db19)
- [Part 2: Motors](#howto-motors-db19)
- [Part 3: Bottom chassis](#howto-bottom-chassis-db19)
- [Part 4: Computation unit](#howto-comp-unit-db19)
- [Part 5: Camera](#howto-camera-db19)
- [Part 6: Top chassis](#howto-top-chassis-db19)
- [Part 7: Back bumper](#howto-back-bumper-db19)
- [Part 8: Front bumper](#howto-front-bumper-db19)
- [Part 9: Battery and Duckie](#howto-battery-duckie-db19) -->

## Unboxing your first Duckiebot {#unboxing-db-beta}

<div figure-id="fig:duckiebox-db-beta" figure-caption="Duckiebox as received.">
     <img src="duckiebox_db-beta.jpg" style='width: 37em' />
</div>

<div figure-id="fig:opened_box_db-beta" figure-caption="First look inside the box.">
     <img src="opened_box_db-beta.jpg" style='width: 37em' />
</div>

Breakdown of box contents (check that you have all of these before proceeding): 
<div figure-id="fig:what_is_in_box_db-beta" figure-caption="What's in the box.">
     <img src="what_is_in_box_db-beta.jpg" style='width: 37em' />
</div>

1. 1x Jetson Nano Board
2. 1x Duckiebot Hut (must have blue sticker on, if not, contact a TA)
3. 1x Camera with camera cable
4. 1x 3D printed camera mount
5. 1x SD Card
6. 1x SD Card reader/Adapter
7. 1x Small screwdriver
8. 1x Bottom Plate
9. 1x Top Plate
10. 4x Motor mount brackets (Note: there are 8 mount brackets in the box, 4 coming in the bag with the top and bottom plate, and additional 4 included with the motor. Use only the 4 included with the motor).
11. 1x Duckie Battery (Black or Gray)
12. 2x Blue motors
13. 2x Motor cables
14. 2x Wheels
15. 1x Front LED panel (3 LEDs)
16. 1x Back LED panel (2 LEDs)
17. 2x Female cable connectors (each containing a white, black and red cable)
18. 1x Metal ball roller
19. 1x Fan
20. 2x Back bumper spacer (red)
21. 2x Rubber Duckies
22. 6x Road signs 
23. 1x Ethernet cable
24. 2x Battery cables - USB (one short black one, and one longer white one)
25. 2x Additional battery charging cables (black ones, on top right of image)
26. 1x Back bumper plate
27. 1x Back bumper plate sticker 
28. 4x zip ties (2 black and 2 white)
29. 1x Duckietown Decal
30. Many - screws, bolts and nuts
31. Double tape (for Duckie City)
32. Road sign skeleton (for Duckie City)
33. Wifi dongle (TP-link)
NOTE: An additional metal spacer has been included which is used to mount the hut (shown below).

## Preliminary Steps {#preliminaries-db-beta}
Important assembly considerations:
- Never assemble the Duckiebot on a metal surface. The pins on the bottom of the board are conductive, and might short. The Duckiebot will be very sad (fried duck).
- Do NOT connect the battery cables to the Jetson Nano board or the hut until the very end. Doing it too early has the potential of causing irreparable damage to the Duckiebot (physically and mentally). 
- All electrical components (Jetson Nano, Board, Hut)  should ALWAYS be connected with nylon screws and nuts.
### Step A
Unbox all of your components, and lay them out on a flat surface. Ensure that you have well lit, uncluttered space to work on.

Although not necessary, you might find useful getting a small (M2.5) wrench to ease some of the passages.

### Step B
Take the rear bumper bracers and the back bumper. The back bumper will be mounted in the last steps as a press fit to the rear bumper bracers. Try to fit the bracers into the holes of the back bumper.

Some bumper bracers have a plastic protective film which is a residue of the manufacturing process. If you struggle in the press fitting, peel off the plastic cover from *one* side of the bracer. If this does not help, peel off the plastic cover from both sides.

### Step C
Peel the plastic cover from all the chassis parts (except the bumper bracers) on both sides.

### Step D
Note that a few among all of your metal screws are special. They are "countersunk" screws. Keep these aside. They will be needed in when assembling the back bumper.

<div figure-id="fig:countersunk-screws-db-beta" figure-caption="Countersunk screws have a tapered head.">
     <img src="countersunk-screws-db18.jpg" style='width: 25em'/>
</div>


### Step E
Every time you read M3x8 screw, a M3x10 will do the same trick. You can exchange them at will.

Do not exchange metal and nylon screws though. The latter are not electrically conductive and are passive protections to potential short circuits that can damage your Duckiebot beyond repair. 

Remember that instructions are your friend! At least at this stage, try to follow them precisely.

If regardless of this foreword you still choose to try and figure it out yourself, it's ok, but for the love of what is precious, *do not plug the battery in until you have performed a visual inspection* [here](#db19-assembly-visual-inspection).  


## Installing the motors {#howto-motors-db-beta}

<!-- <div class='requirements' markdown="1">

Requires: 

<div figure-id="motor_assembly_components">
     <img src="motor_assembly_components.jpg" style='width: 25em'/>
</div>

- 4x Long metal screws (longest in the box)
- 2x Motor cables
- 2x Blue motors
- 4x M3 nuts (the only size available for metal nuts)
- 1x Bottom plate
- 4x motor mounts (Note, the motors mounts that come with the bottom plate have very tight tolerancing. For easier installation, use the additional motor mounts that come in the same bag with one of the motors).


Results: <div figure-id="fig:countersunk-screws-db19" figure-caption="Countersunk screws have a tapered head.">
     <img src="countersunk-screws-db18.jpg" style='width: 25em'/>
</div>

</div> -->

<!-- TODO: it would be nice to add a knowledge and activity graph at the beginning of each step -->

Needs:

- 4x Long metal screws (longest in the box)
- 2x Motor cables
- 2x Blue motors
- 4x M3 nuts (the only size available for metal nuts)
- 1x Bottom plate
- 4x motor mounts (Note, the motors mounts that come with the bottom plate have very tight tolerancing. For easier installation, use the additional motor mounts that come in the same bag with one of the motors).

<div figure-id="fig:motor_assembly_components">
     <img src="motor_assembly_components.jpg" style='width: 25em'/>
</div>

### Connect the motor cables to the motors

<div figure-id="fig:motors_db-beta">
     <img src="motors_db-beta.jpg" style='width: 25em'/>
</div>

### Place the inner motor mounts on the bottom plate

<div figure-id="fig:bottomplate_motor_supports">
     <img src="bottomplate_motor_supports.jpg" style='width: 25em'/>
</div>

### Connect the first motor
Use the screws and nuts to connect the first motor (note, cable must be attached already). A good idea is to place both screws first to help with positioning, and the start with the top nut. 

<div figure-id="fig:motor_backplate">
     <img src="motor_backplate.jpg" style='width: 25em'/>
</div>
<div figure-id="fig:one_motor_attached">
     <img src="one_motor_attached.jpg" style='width: 25em'/>
</div>

### Connect the second motor

<div figure-id="fig:motors_installed_bottom_plate">
     <img src="motors_installed_bottom_plate.jpg" style='width: 25em'/>
</div>


## Attaching the back rolling ball {#how-to-back-rolling-ball-dbbeta}

Needed:
- 1x metal rolling ball
- 2x 25mm metal spacers (shorter ones)
- 4x  M3x10 screws (shortest metal screws in the box)
- 1x Previous [assembly](fig:motors_installed_bottom_plate) or [assembly](#howto-motors-db-beta).


<div figure-id="fig:roller_components">
     <img src="roller_components.jpg" style='width: 25em'/>
</div>

### Connect the metal spacers to the bottom plate
Use 2x M3x10 metal screws to connect the metal spacers to the bottom plate.

<div figure-id="fig:roller_spacers">
     <img src="roller_spacers.jpg" style='width: 25em'/>
</div>
 
### Attach the rolling ball

Attach the metal rolling ball using 2x M3x10 metal screws. Motor cable management is not final yet, but keeping it between the spacers is a good idea for later. 


<div figure-id="fig:wheel_roller">
     <img src="wheel_roller.png" style='width: 25em'/>
</div>

## Adding rear mount bracket

Needed:

* 2x M3x10 screws 
* 2x 25mm spacers (shorter)
* 1x back acrylic spacer
* 1x Previous [assembly](#how-to-back-rolling-ball-dbbeta)

<div figure-id="fig:bumper_spacers">
     <img src="bumper_spacers.jpg" style='width: 25em'/>
</div>

Use 2x M3x10 metal screws to attach the metal spacer to the back side of the Duckiebot. The red spacer should be placed in between the bottom plate and the metal spacers. 

<div figure-id="fig:bottom_back_bumper_spacer">
     <img src="bottom_back_bumper_spacer.jpg" style='width: 25em'/>
</div>

<div figure-id="fig:motor_cables_and_roller">
     <img src="motor_cables_and_roller.jpg" style='width: 25em'/>
</div>






  
