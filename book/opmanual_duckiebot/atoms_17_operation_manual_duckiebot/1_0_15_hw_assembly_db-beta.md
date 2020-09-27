# Assembling the `DB-beta` Duckiebot {#assembling-duckiebot-db-beta status=ready}

This page is for the `DB-beta` configuration used in the AMoD class at ETH Zurich in 2020. The hardware used in this Duckiebot model is very similar to the hardware of the `DB19`. The major difference between these models is the replacement of the Raspberry Pi 3 by a Jetson Nano Nvidia board which lead to other minor hardware changes like a new camera, a cooling fan, layout, etc.

<div class='requirements' markdown="1">

Requires: Be enrolled in ETHZ’s AMoD 2020 course and have picked up your Duckiebox from the Autolab.

Requires: Time: about 1-1.5 hours (45 minutes for an experienced Duckiebot builder).

Results: An assembled Duckiebot in configuration `DB-beta`.

</div>

<!-- Note: The [FAQ](#op-faq-db19) section at the bottom of this page may already answer some of you comments, questions or doubts. -->


There are 12 parts in this procedure. Most parts build upon previous steps, so make
sure to follow them in the following order:

- [Part 1: Unboxing](#unboxing-db-beta) 
- [Part 2: Preliminary steps](#preliminaries-db-beta)
- [Part 3: Installing the motors](#howto-motors-db-beta)
- [Part 4: Attaching the back rolling ball](#how-to-back-rolling-ball-dbbeta)
- [Part 5: Adding the rear mount bracket](#adding_rear_mount_bracker_dbbeta)
- [Part 6: Adding the front spacers](#adding_front_spacers_dbbeta)
- [Part 7: Battery Attachment](#battery_attachment_dbbeta)
- [Part 8: Camera assembly and attachment](#camera_assembly_and_attachment_dbbeta)
- [Part 9: Top plate assembly](#top_plate_assembly_dbbeta) 
- [Part 10: Back bumper assembly](#back_bumper_assembly_dbbeta) 
- [Part 11: Hut attachment](#hut_attachment_dbbeta) 
- [Part 12: Final assembly - putting it all together](#final_assembly_dbbeta) 

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

<div figure-id="fig:additional_nut_screw_spacer1">
     <img src="additional_nut_screw_spacer.jpg" style='width: 25em'/>
</div>

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

Needs:

- 4x Long metal screws (longest in the box)
- 2x Motor cables
- 2x Blue motors
- 4x M3 nuts (the only size available for metal nuts)
- 1x Bottom plate
- 4x motor mounts (Note, the motors mounts that come with the bottom plate have very tight tolerancing. For easier installation, use the additional motor mounts that come in the same bag with one of the motors).

<div figure-id="fig:motor_assembly_components" figure-caption="Needed components for this step.">
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

Needs:

- 1x metal rolling ball
- 2x 25mm metal spacers (shorter ones)
- 4x  M3x10 screws (shortest metal screws in the box)
- 1x Previous [](#fig:motors_installed_bottom_plate) or [assembly](#howto-motors-db-beta).


<div figure-id="fig:roller_components" figure-caption="Needed components for this step.">
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

## Adding rear mount bracket {#adding_rear_mount_bracker_dbbeta}

Needed:

* 2x M3x10 screws 
* 2x 25mm spacers (shorter)
* 1x back acrylic spacer
* 1x Previous [assembly](#how-to-back-rolling-ball-dbbeta)

<div figure-id="fig:bumper_spacers" figure-caption="Needed components for this step.">
     <img src="bumper_spacers.jpg" style='width: 25em'/>
</div>

Use 2x M3x10 metal screws to attach the metal spacer to the back side of the Duckiebot. The red spacer should be placed in between the bottom plate and the metal spacers. 

<div figure-id="fig:bottom_back_bumper_spacer">
     <img src="bottom_back_bumper_spacer.jpg" style='width: 25em'/>
</div>

<div figure-id="fig:motor_cables_and_roller">
     <img src="motor_cables_and_roller.jpg" style='width: 25em'/>
</div>


## Adding the front spacers {#adding_front_spacers_dbbeta}

Needed:

- 2x M3x10 screws
- 2x 30mm metal spacers (only place where the longer ones are used)
- 1x Previous [assembly](#adding_rear_mount_bracker_dbbeta)

<div figure-id="fig:roller_spacer_components" figure-caption="Needed components for this step.">
     <img src="roller_spacer_components.jpg" style='width: 25em'/>
</div>


Use 2x M3x10 metal screws to attach the front spacers. The location should be as shown on the picture (at the bottom of the outer “T”s in the front)


<div figure-id="fig:bottom_plate_complete">
     <img src="bottom_plate_complete.jpg" style='width: 25em'/>
</div>


## Battery Attachment {#battery_attachment_dbbeta}

Needed:

- 1x Battery
- 1x Zip tie (long white one)
- 1x Back acrylic spacer
- 4x M3x10 screws
- 1x Previous [assembly](#adding_front_spacers_dbbeta)

<div figure-id="fig:attach_battery_components" figure-caption="Needed components for this step.">
     <img src="attach_battery_components.jpg" style='width: 25em'/>
</div>


<div figure-id="fig:batteries_db-beta" figure-caption="The two types of batteries that might come with the Duckiebox.">
     <img src="batteries_db-beta.jpg" style='width: 25em'/>
</div>


Place the battery on the bottom plate with the ports facing to the rear left. Attach it firmly with the longer white zip tie to prevent it from moving and touching the wheels. Make sure to place the zip-tie to the side so that it does not block the top plate from being placed later.


<div figure-id="fig:battery_strap_dbbeta">
     <img src="battery_strap_dbbeta.jpg" style='width: 25em'/>
</div>


<div figure-id="fig:motors_battery_attached">
     <img src="motors_battery_attached.jpg" style='width: 25em'/>
</div>

<div figure-id="fig:battery_strap_detail">
     <img src="battery_strap_detail.jpg" style='width: 25em'/>
</div>


## Camera assembly and attachment {#camera_assembly_and_attachment_dbbeta}

### Camera assembly {#camera_assembly_dbbeta}

Needed:

- 1x Camera 
- 1x 3D printed camera mount
- 1x Camera cable
- 4x M2.5x10mm nylon screws (the smallest nylon screws)
- 4x M2.5 nylon nuts 

<div figure-id="fig:camera_assembly_components_beta" figure-caption="Needed components for this step.">
     <img src="camera_assembly_components_beta.jpg" style='width: 25em'/>
</div>

Use the nylon screws to mount the camera directly to the camera mount. Tighten using the nylon nuts. The camera cable should be placed upwards. The correct placement of the camera is pointing slightly towards the ground. 

<div figure-id="fig:camera_cable">
     <img src="camera_cable.jpg" style='width: 25em'/>
</div>

### Camera attachment {#camera_attachment_dbbeta}

Needed:

- 1x top plate
- 2x triple-connector cables
- 1x [Camera assembly](#camera_assembly_dbbeta)
- 3x M3x10 metal screws
- 3x M3 metal nuts

<div figure-id="fig:camera_attachment_components" >
     <img src="camera_attachment_components.jpg" style='width: 25em'/>
</div>

<div figure-id="fig:camera_attached" figure-caption="Mount the camera mount onto the middle plate using the 3x M3x10 screws. ">
     <img src="camera_attached.jpg" style='width: 25em'/>
</div>


<div figure-id="fig:camera_top_plate" figure-caption="Place the connecting cables (connecting front and back plate) together as shown in the picture.">
     <img src="camera_top_plate.jpg" style='width: 25em'/>
</div>

## Top plate assembly {#top_plate_assembly_dbbeta}

Needed:
- 1x Jetson Nano board
- 4x  M2.5 nylon nuts
- 4x M2.5x10 nylon screws
- 4x Nylon female screw spacer (1 size only)
<!-- - 1x micro sd (flashed, instructions: https://docs.duckietown.org/daffy/opmanual_duckiebot/out/setup_duckiebot.html ) -->
- 1x Previous [assembly](#camera_attachment_dbbeta)


 <div figure-id="fig:top_plate_assembly_components_beta" figure-caption="Needed components for this step.">
     <img src="top_plate_assembly_components_beta.jpg" style='width: 25em'/>
</div>

### Step 1 

Place the nylon screws in the locations shown below. Screw is inserted from the bottom. 

<div figure-id="fig:bumper_cables_layout" >
     <img src="bumper_cables_layout.jpg" style='width: 25em'/>
</div>


### Step 2

Secure the nylon screws from the top using the M2.5 nylon nuts (x4). Do not tighten yet, as it might be necessary to adjust the location of the screws to fit the Jetson Nano board. 


<div figure-id="fig:nylon_screws_for_jetson" >
     <img src="nylon_screws_for_jetson.jpg" style='width: 25em'/>
</div>

### Step 3

Place the Jetson Nano board on top, and secure with Nylon female spacers. 

<div figure-id="fig:jetson_attachment" >
     <img src="jetson_attachment.jpg" style='width: 25em'/>
</div>

<div figure-id="fig:jetson_installed_top_plate" >
     <img src="jetson_installed_top_plate.jpg" style='width: 25em'/>
</div>


## Back bumper assembly {#back_bumper_assembly_dbbeta}

Needed:

- 1x back bumper (red, with 2 LEDs)
- 1x back bumper tag
- 1x back bumper plate
- 2x 25mm metal spacer
- 2x M3x8 metal screws (shortest metal screws)
- 2x M3x10 metal countersink screw

 <div figure-id="fig:back_bumper_components" figure-caption="Needed components for this step.">
     <img src="back_bumper_components.jpg" style='width: 25em'/>
</div>

Assemble until you have something that looks like the pictures below. 

<div figure-id="fig:back_bumper_connected" >
     <img src="back_bumper_connected.jpg" style='width: 25em'/>
</div>

<div figure-id="fig:back_bumperplate_assembled" >
     <img src="back_bumperplate_assembled.jpg" style='width: 25em'/>
</div>

## Hut attachment {#hut_attachment_dbbeta}

Needed:

- 1x Hut (with blue sticker)
- 1x M2.5x10 nylon screw
- 1x M2.5 nylon nut
- 1x 12mm metal spacer

<div figure-id="fig:hut_bottom_dbbeta" figure-caption="Needed components for this step.">
     <img src="hut_bottom_dbbeta.jpg" style='width: 25em'/>
</div>

<div figure-id="fig:additional_nut_screw_spacer_hut" figure-caption="Needed components for this step.">
     <img src="additional_nut_screw_spacer.jpg" style='width: 25em'/>
</div>

Mount the single spacer in the location shown below. 


<div figure-id="fig:hut_spacer" >
     <img src="hut_spacer.jpg" style='width: 25em'/>
</div>



## Final assembly - putting it all together {#final_assembly_dbbeta}

### Top and bottom plate assembly

Needed:

- 1x [Top plate assembly](#top_plate_assembly_dbbeta)
- 1x Bottom plate assembly, as seen in [](#fig:battery_strap_dbbeta) with wheels attached
- 1x [Back bumper assembly](#back_bumper_assembly_dbbeta)
- 2x Battery Cables (1x short black cable, 1x short white cable)
- 1x Back plate spacer
- 1x Front bumper (with 3 LEDs)
- 4x M3x10 metal screws 
- 1x Hut
- 2x M2.5 nylon nuts (for hut)

<div figure-id="fig:top_plate_components" figure-caption="Needed components for this step.">
     <img src="top_plate_components.jpg" style='width: 25em'/>
</div>

Place the back spacer in between the bottom and top plate assembly and secure using 2x M3x10 metal screws. 

<div figure-id="fig:attach_back_bumper_plate" >
     <img src="attach_back_bumper_plate.jpg" style='width: 25em'/>
</div>

Use additional 2x M3x10 metal screws to secure the Top Plate assembly to the Bottom Plate assembly in the front (close to the camera). Leave the front screws a bit loose so that the front bumper can be fit between the two plates.

<div figure-id="fig:top_plate_screw" >
     <img src="top_plate_screw.jpg" style='width: 25em'/>
</div>

Mount the Hut Assembly (with metal spacer) onto the back of the Duckiebot. Secure with 2x M2.5 Nylon nuts. 

<div figure-id="fig:nylon_things_hut" >
     <img src="nylon_things_hut.jpg" style='width: 25em'/>
</div>

<div figure-id="fig:hut_install_components" >
     <img src="hut_install_components.jpg" style='width: 25em'/>
</div>


Connect the camera to the “CAM 0”, in case of more than one camera connector on the Jetson Nano Board. Secure with a Zip tie (short white one). 

<div figure-id="fig:camera_cable_connection_direction" >
     <img src="camera_cable_connection_direction.jpg" style='width: 25em'/>
</div>

<div figure-id="fig:camera_cable_connection" >
     <img src="camera_cable_connection.jpg" style='width: 25em'/>
</div>

<div figure-id="fig:camera_cable_strapped" >
     <img src="camera_cable_strapped.jpg" style='width: 25em'/>
</div>

### Adding Bumpers and Connecting wires

Needed:

- 1x Front bumper (with 3 LED)
- 1x Back bumper assembly
- 4x M3x8 metal phillips head screw
- 4x M3 metal nuts

Follow connecting guidelines. 

Warning: Please triple check that all connections follow this guideline, otherwise your front or back bumpers might get fried.

- Red (5V)
- DIN / DOU (white)
- Ground (black)
 
<div figure-id="fig:bumper_plates" figure-caption="Needed components for this step.">
     <img src="bumper_plates.jpg" style='width: 25em'/>
</div>

Connect front LEDs to the hut. Grab the wires coming from the front bumper, and connect them to the hut as shown below. 

<div figure-id="fig:led_hut_connection" >
     <img src="led_hut_connection.jpg" style='width: 25em'/>
</div>

Connect motor cables to the Hut. Right motor corresponds to the right space in the hut. There’s only one possible orientation for the cable connection, don’t force it. 

<div figure-id="fig:motor_cables_dbbeta" >
     <img src="motor_cables_dbbeta.jpg" style='width: 25em'/>
</div>

Connect the back bumper cables to the main Duckiebot

<div figure-id="fig:back_bumper_dbbeta" >
     <img src="back_bumper_dbbeta.jpg" style='width: 25em'/>
</div>

Place the back bumper using 2x or 4x metal screws. 

<div figure-id="fig:hut_spacer_iso" >
     <img src="hut_spacer_iso.jpg" style='width: 25em'/>
</div>

Connect Front bumper to the Duckiebot
 
<div figure-id="fig:front_bumper_cables" >
     <img src="front_bumper_cables.jpg" style='width: 25em'/>
</div>

<div figure-id="fig:led_connection_front_led_plate" >
     <img src="led_connection_front_led_plate.jpg" style='width: 25em'/>
</div>

<div figure-id="fig:led_front_plate_connection" >
     <img src="led_front_plate_connection.jpg" style='width: 25em'/>
</div>

Gently loosen the front screws connecting the top plate and bottom plate. Lift the top plate and slide the front bumper into the front:

<div figure-id="fig:top_plate_front_attachment" >
     <img src="top_plate_front_attachment.jpg" style='width: 25em'/>
</div>

<div figure-id="fig:camera_attached2" >
     <img src="camera_attached2.jpg" style='width: 25em'/>
</div>

<div figure-id="fig:camera_attached3" >
     <img src="camera_attached3.jpg" style='width: 25em'/>
</div>

Screw the top plate back. 


<div figure-id="fig:front_bumper_dbbeta" >
     <img src="front_bumper_dbbeta.jpg" style='width: 25em'/>
</div>

### Connecting the fan and finishing up
Needed:

- 1x Duckiebot
- 1x Fan
- 4x M3x12 metal screws (longer screws)
- 2x Battery Cables (1x short black cable, 1x short white cable)
- 1x Duckie 

Screw the fan on top of the heatsink of the Jetson Nano using the long M3x12 metal screws. 

<div figure-id="fig:finished_view_iso" >
     <img src="finished_view_iso.jpg" style='width: 25em'/>
</div>

Connect the fan pins to the bottom pins under the hut (pair of pins closest to the back bumper). 

Warning: Make sure the black cable is connected to GND and the red one to VCC (5V).

<div figure-id="fig:fan_connection" >
     <img src="fan_connection.jpg" style='width: 25em'/>
</div>

Voila! Final result should look something like this: 


<div figure-id="fig:final_result_topview" >
     <img src="final_result_topview.jpg" style='width: 25em'/>
</div>

<div figure-id="fig:final_result_left" >
     <img src="final_result_left.jpg" style='width: 25em'/>
</div>


<div figure-id="fig:finished_view1" >
     <img src="finished_view1.jpg" style='width: 25em'/>
</div>

<div figure-id="fig:front_bumper_iso_view" >
     <img src="front_bumper_iso_view.jpg" style='width: 25em'/>
</div>

<div figure-id="fig:back_bumperplate" >
     <img src="back_bumperplate.jpg" style='width: 25em'/>
</div>

<div figure-id="fig:finished_view2" >
     <img src="finished_view2.jpg" style='width: 25em'/>
</div>

Congratulations, you completed the easiest part of the course! Now your journey to Mount Doom begins...