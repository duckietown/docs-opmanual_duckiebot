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

## Foreword:
Important assembly considerations:
- Never assemble the Duckiebot on a metal surface. The pins on the bottom of the board are conductive, and might short. The Duckiebot will be very sad (fried duck).
- Do NOT connect the battery cables to the Jetson Nano board or the hut until the very end. Doing it too early has the potential of causing irreparable damage to the Duckiebot (physically and mentally). 
- All electrical components (Jetson Nano, Board, Hut)  should ALWAYS be connected with nylon screws and nuts.

## Unboxing your first Duckiebot

<div figure-id="fig:duckiebox-db-beta" figure-caption="Duckiebox as received.">
     <img src="duckiebox-db-beta.png" style='width: 37em' />
</div>

<div figure-id="fig:opened_box_db-beta" figure-caption="First look inside the box.">
     <img src="opened_box_db-beta.png" style='width: 37em' />
</div>

Breakdown of box contents (check that you have all of these before proceeding): 
<div figure-id="fig:opened_box_db-beta" figure-caption="What's in the box.">
     <img src="what_is_in_box_db-beta.png" style='width: 37em' />
</div>

- [ ] 1. 1x Jetson Nano Board
- [ ] 2. 1x Duckiebot Hut (must have blue sticker on, if not, contact a TA)
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


## Installing the motors

<div class='requirements' markdown="1">

Requires: 
- 4x Long metal screws (longest in the box)
- 2x Motor cables
- 2x Blue motors
- 4x M3 nuts (the only size available for metal nuts)
- 1x Bottom plate
- 4x motor mounts (Note, the motors mounts that come with the bottom plate have very tight tolerancing. For easier installation, use the additional motor mounts that come in the same bag with one of the motors).


Results: Insert a picture

</div>


