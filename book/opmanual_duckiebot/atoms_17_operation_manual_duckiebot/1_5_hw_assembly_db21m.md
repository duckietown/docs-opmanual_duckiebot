# Assembly - Duckiebot `DB21M` {#assembling-duckiebot-db21m status=ready}

<!--

<div figure-id="fig:Duckiebook-Banner" figure-caption="The Duckiebot MOOC Founder's Edition, powered by NVIDIA Jetson Nano.">
     <img src="Duckiebook_Banner.png" style='width: 40em' />
</div>

-->

<div class='requirements' markdown="1">

Requires: Duckiebot `DB21M` parts (get them [here](https://get.duckietown.com/)). Here is an [overview of all existing configurations](#duckiebot-configurations).

Requires: A microSD card with the Duckiebot image on it. The procedure to flash the SD card is explained [here](#setup-duckiebot).

Requires: 3 hours of assembly time.

Result: An assembled Duckiebot in configuration `DB21M`.

</div>

<!--
-->
These instructions are your friend. Follow them carefully, especially if it's the first time you assemble a `DB21M`. Small variations might cause big effects (e.g., don't flip your cables around!).

The troubleshooting section

Overview of all parts in your Duckiebox:

<div figure-id="fig:db21m-parts-overview">
     <img src="db21-parts-overview.jpg" style='width: 40em' />
</div>


The assembly process is divided in 9 parts. They must be completed in the following order:

- [Part 1: Preliminary Steps](#howto-preliminary-db21m)
- [Part 2: Drive Train](#howto-drive-train-db21m)
- [Part 3: Battery Pack Installation](#howto-battery-pack-installation-db21m)
- [Part 4: Computational Unit and Rear Assembly](#howto-comp-unit-db21m)
- [Part 5: Cable Management](#howto-cables-db21m)
- [Part 6: Front Assembly](#howto-front-assembly-db21m)
- [Part 7: Top Plate Assembly](#howto-interactive-cover-db21m)
- [Part 8: Power your Duckiebot](#howto-power-db21m)
- [Part 9: Additional Parts](#howto-additional-parts-db21m)

- [FAQ](#op-faq-db21m)

The FAQ section at the bottom of this page might be the first thing to visit if something is not clear. This may already answer your comments, questions or doubts.

## Preliminary Steps {#howto-preliminary-db21m}

### Unboxing
Unbox all of your components and lay them out on a flat surface. Ensure that you have well lit, uncluttered space to work on. Do not show any violence to the Duckiebot pieces or they might break.

Note: Your Duckiebot chassis are under the white protection foam. To get your Duckiebot chassis, pull out the white protection foam in the box after remove everything.

Although not necessary, a small (M2.5) wrench will ease some of the next passages.

### Plastic cover
Peel the plastic cover from all the chassis parts on both sides.

### Screws, Nuts and Stand-offs
Each type of screw, nut and stand-off is labeled with an index. You will find corresponding labels on the pictures at each step. Using the right parts at each step will prevent undesirable effects (e.g., nylon screws prevent electrical shorts).

<div figure-id="fig:db21m-parts_indices" figure-caption="Overview of interlocking parts">
     <img src="db21-parts_indices.png" style='width: 40em' />
</div>


## Drive Train {#howto-drive-train-db21m}

### Step 0 - Charge battery via `HUT`
- Connect the battery and the HUT board as shown and make sure a green LED on the HUT is lit.
- Wait 30 minutes and then push the button on the battery.
- Check that the state of charge LEDs on the battery start blinking.
- Leave this setup until the battery is fully charged. This may take up to 5 hours.

<div figure-id="fig:db21m-step_00">
     <img src="db21-step_00.png" style='width: 40em' />
</div>
In the following steps (1 to 12) you will build the base plate assembly.

<div figure-id="fig:db21m-overview-step_01-12">
     <img src="db21-overview-step_01-12.png" style='width: 30em' />
</div>


### Step 1
Take the base plate (number `02` on the bottom side) and insert two metal M3 nuts (`N3`) as shown.

It might appear tricky at first to make the nuts fit. Once on the inset, gently rotate each nut until it falls in place (note the final orientation of the screw in the picture).

<div figure-id="fig:db21m-step_01">
     <img src="db21-step_01.png" style='width: 40em' />
</div>

Note: Occasionally manufacturing tolerances (on the nut and the laser cut chassis) might prevent a flush fit. Trying a different nut or changing the insert direction might solve the problem.

### Step 2
Check the hole pattern in the middle of the plate to make sure you are holding it the right orientation, then insert two of the motor plates in the base plate.

<div figure-id="fig:db21m-step_02">
     <img src="db21-step_02.png" style='width: 40em' />
</div>

### Step 3
Connect one of the 6-pin motor cable to the blue motor.

<div figure-id="fig:db21m-step_03">
     <img src="db21-step_03.png" style='width: 40em' />
</div>

Note: It might be easier if you place the base plate on a flat surface so hold the motor plates. Use one hand to hold the nut, and the other to drive the screwdriver.

### Step 4
Place the motor between the two plates and tighten it with two M3x30 screws (`B5`) and two metal M3 nuts (`N3`). Tighten the screws enough so that the final assembly does not wobble. Don't use excessive force to avoid getting hurt (or braking something).

<div figure-id="fig:db21m-step_04">
     <img src="db21-step_04.png" style='width: 40em' />
</div>

Note: Pre-bend the cable before carefully passing it through the cable management inset on the chassis. Don't use force to avoid braking the inset.

### Step 5
Connect the second 6-pin motor cable to the second motor.

<div figure-id="fig:db21m-step_05">
     <img src="db21-step_05.png" style='width: 30em' />
</div>

### Step 6
Insert the other two motor plates into the base plate, similarly to step 2.

<div figure-id="fig:db21m-step_06">
     <img src="db21-step_06.png" style='width: 40em' />
</div>

### Step 7
Tighten the second motor with two M3x30 metal screws (`B5`) and two metal M3 nuts (`N3`) and place the cable, similarly to steps 3 and 4.

<div figure-id="fig:db21m-step_07">
     <img src="db21-step_07.png" style='width: 40em' />
</div>

### Step 8
Connect one of the longest 4-pin cables to the white connector of the IMU (Inertial Measurement Unit) board.

<div figure-id="fig:db21m-step_08">
     <img src="db21-step_08.png" style='width: 40em' />
</div>

### Step 9
Attach the IMU board to the base plate using two nylon M2.5x10 screws (`B2`) and two nylon M2.5 nuts (`N2`).

<div figure-id="fig:db21m-step_09">
     <img src="db21-step_09.png" style='width: 40em' />
</div>

### Step 10
Flip the assembly and verify that the three cables are routed through their corresponding holes or slits (cable of the *left* motor through the *left* slit, cable of the *right* motor through the *right* slit).

<div figure-id="fig:db21m-step_10">
     <img src="db21-step_10.png" style='width: 40em' />
</div>

### Step 11
Flip the assembly again and mount the two M3x25 stand-offs (`S3`) to the bottom plate. Use two metal M3x8 screws (`B3`).

<div figure-id="fig:db21m-step_11">
     <img src="db21-step_11.png" style='width: 40em' />
</div>

### Step 12
Mount the omni-wheel to the stand-offs using two of the metal M3x8 screws (`B3`).

<div figure-id="fig:db21m-step_12">
     <img src="db21-step_12.png" style='width: 40em' />
</div>

Note: If you note the screws don't go all the way, try flipping the stand-off.

### Verify the assembly

Before proceeding, verify that no component is wiggling. The only things moving should be the cables and the sphere in the omni-wheel (and, yes, the motor axles). Proceed to gently tightening the screws of the offending parts, if necessary.

Congratulations, you just built the base plate Duckiebot assembly!

## Battery Pack Installation {#howto-battery-pack-installation-db21m}
The following steps (13 to 18) guide through the *Duckibattery* assembly:

<div figure-id="fig:db21m-overview-step_13-18">
     <img src="db21-overview-step_13-18.png" style='width: 30em' />
</div>

<!--
You can try to mount the wheels even without the distance disks. But make sure that the wheels do not touch the frame of the Duckiebot when turning.

<div figure-id="fig:db21m-step_22B">
     <img src="db21-step_22B.png" style='width: 40em' />
</div>

-->

### Step 13
Take the upper plate (`01`) and 8 metal M3 nuts (`N3`). Compare the hole in the green circles with the hole position on your plate and make sure they agree (if the number `01` is visible on top, you are good to go).

<div figure-id="fig:db21m-step_13">
     <img src="db21-step_13.png" style='width: 40em' />
</div>

### Step 14
Insert 4 nylon M2.5x10 screws (`B2`) from the top and tighten them with 4 nylon M2.5 nuts (`N2`).

<div figure-id="fig:db21m-step_14">
     <img src="db21-step_14.png" style='width: 40em' />
</div>

### Step 15
Take the two plates with numbers `04L` and `04R` and insert three metal M3 nuts (`N3`) into each of them.

<div figure-id="fig:db21m-step_15">
     <img src="db21-step_15.png" style='width: 40em' />
</div>

### Step 16
Connect these two plates to the upper plate with a metal M3x8 screw (`B3`) each. Note the slightly different holes in the side plates to mount them in the right way!

<div figure-id="fig:db21m-step_16">
     <img src="db21-step_16.png" style='width: 40em' />
</div>

### Step 17
Insert the battery between the two side plates. Make sure the Mack the duck is on the bottom. The USB ports of the battery are not the same. Flipping the battery at this stage will cause the Duckiebot not to power on.

<div figure-id="fig:db21m-step_17">
     <img src="db21-step_17.png" style='width: 40em' />
</div>

<div figure-id="fig:db21m-step_17additional">
     <img src="db21-step_17additional.png" style='width: 20em' />
</div>

### Step 18
Take the two small plates labeled `07` and lock the battery in place.

<div figure-id="fig:db21m-step_18">
     <img src="db21-step_18.png" style='width: 40em' />
</div>


## Computation Unit and Rear Assembly {#howto-comp-unit-db21m}
At this point, we are starting to see the final shape of the Duckiebot! The following steps (19 to 27) will help assemble the lower frame and mount the NVIDIA Jetson Nano board:

<div figure-id="fig:db21m-overview-step_19-27">
     <img src="db21-overview-step_19-27.png" style='width: 30em' />
</div>

### Step 19
Take the lower half of the Duckiebot from steps 1 to 12 again and mount it directly to the assembly you have just created using 4 metal M3x8 (`B3`) screws. Make sure all the plates are locked in place correctly.

<div figure-id="fig:db21m-step_19">
     <img src="db21-step_19.png" style='width: 40em' />
</div>

Note: This is a tricky step and might require a few tries to get it right. Remember that the Roboticist is patient and welcomes challenges with joy and determination!

### Step 20
Check the routing path of the two motor cables again from step 10 and wire them through the holes in the upper plate.

<div figure-id="fig:db21m-step_20">
     <img src="db21-step_20.png" style='width: 40em' />
</div>

### Step 21
Do a similar procedure for the cable of the IMU unit.

<div figure-id="fig:db21m-step_21">
     <img src="db21-step_21.png" style='width: 40em' />
</div>

### Step 22
Take the two yellow driving wheels and push them to the motors using one distance disk (`S4`) between each of them.

<div figure-id="fig:db21m-step_22">
     <img src="db21-step_22.png" style='width: 40em' />
</div>

### Step 23
1. Insert the SD card to the slot on the NVIDIA Jetson Nano board. Make sure the metal pins of the SD card are facing the heat sink.

2. Place the Jetson Nano on the 4 screws from step 14 but DO NOT tighten the Jetson with any nuts or stand-offs yet!

<div figure-id="fig:db21m-step_23">
     <img src="db21-step_23.png" style='width: 40em' />
</div>


### Step 24
Take the side cover carrying the number `05L`. Insert a nylon M2.5 nut (`N2`) and a metal M3 nut (`N3`) into the plate (note the engravings: `N2` and `N3` on the plate itself).

Then secure the plate to the frame using two metal M3x8 screws (`B3`).

You may need to lift the Jetson Nano a little to fit the side cover plate under the NVIDIA Jetson Nano board.

<div figure-id="fig:db21m-step_24">
     <img src="db21-step_24.jpg" style='width: 40em' />
</div>

### Step 25
Place 4 of the (`S5`) spacers on the nylon screws holding the NVIDIA Jetson Nano board.

<div figure-id="fig:db21m-step_25">
     <img src="db21-step_25.png" style='width: 40em' />
</div>

### Step 26
Tighten the NVIDIA Jetson Nano board with the 6 brass stand-offs (`S2`). Put two stand-offs on each of the front screws but only one each on the back.

<div figure-id="fig:db21m-step_26">
     <img src="db21-step_26.png" style='width: 40em' />
</div>

### Step 27
Take the side cover carrying the number `05R`. Insert a nylon M2.5 nut (`N2`) and a metal M3 nut (`N3`) into the plate, similarly to step 24 (note the engraving: `N2` and `N3` on the plate itself). Then screw the plate to the frame using two metal M3x8 screws (`B3`).

<div figure-id="fig:db21m-step_27">
     <img src="db21-step_27.png" style='width: 40em' />
</div>

## Cable Management {#howto-cables-db21m}

### Step 28
Take the USB cable that has three connectors. Connect the Duckiebattery and the NVIDIA Jetson Nano board with the USB-A ports as shown in the picture below.

<div figure-id="fig:db21m-step_28">
     <img src="db21-step_28.png" style='width: 40em' />
</div>

Note: The micro USB connector must not be connected at that stage!

### Step 29

Take the angled micro USB to micro USB cable and plug the angled connector to the middle port on the Duckiebattery. Wire the cable through the chassis as shown in the picture below. The micro USB end must be unplugged at that stage!

Note: An experienced Roboticist at this point would mark the free end of this cable (e.g., with a sticker or some tape), to make it distinguishable from the other free roaming cable with a micro USB connector, so to make it very improbable to mix the two up and prevent future headaches!

<div figure-id="fig:db21m-step_29">
     <img src="db21-step_29.png" style='width: 40em' />
</div>

### Step 30
Now take the last USB cable, with a micro USB plug on one side and an angled USB-A plug on the other side. Connect the USB-A end to the last (right) port on the Duckiebattery. Again, wire the cable through the chassis as shown and leave the micro USB connector free on the other side.

<div figure-id="fig:db21m-step_30">
     <img src="db21-step_30.png" style='width: 40em' />
</div>

### Step 31
Connect the Wi-Fi dongle to the upper USB-A port on the NVIDIA Jetson Nano board.

<div figure-id="fig:db21m-step_31">
     <img src="db21-step_31.png" style='width: 40em' />
</div>

### Step 32
Take the back bumper board and connect the 4-pin cable of medium length to the white plug on the board.

<div figure-id="fig:db21m-step_32">
     <img src="db21-step_32.png" style='width: 40em' />
</div>

### Step 33
Wire the cable attached to the back bumper through the same hole in the upper plate as the motor cable of the left driving motor.

<div figure-id="fig:db21m-step_33">
     <img src="db21-step_33.png" style='width: 40em' />
</div>

### Step 34
When attaching the back bumper board to the chassis, make sure the pins of the lower and upper plate all fit well. Tighten the board with three metal screws (`B3`).

<div figure-id="fig:db21m-step_34">
     <img src="db21-step_34.png" style='width: 40em' />
</div>

### Step 35
Take the plate carrying the number `06` and press two nylon M2.5 nuts (`N2`) into the corresponding slits.

<div figure-id="fig:db21m-step_35">
     <img src="db21-step_35.png" style='width: 40em' />
</div>

### Step 36
Mount the plate number `06` to the back of the chassis using two metal M3x8 screws (`B3`).

Note: the `06` is not symmetric and the orientation matters. If the number `06` is pointing towards the Duckiebot, we are good to go!

<div figure-id="fig:db21m-step_36">
     <img src="db21-step_36.png" style='width: 40em' />
</div>

### Step 37
Take the fan and mount it on top of the heat sink of the NVIDIA Jetson Nano board using 4 metal M3x12 screws (`B4`). Make sure the cable of the fan is pointing to the back right side (it might be necessary to use a little force on these screws, as the thread has to cut it's way through the heat sink the first time).

Note: You don't need to tighten the screws completely but the fan must sit tight.

<div figure-id="fig:db21m-step_37">
     <img src="db21-step_37.png" style='width: 40em' />
</div>

### Step 38

Take the PCB with the Duckietown logo on it (we'll call it the `HUT` from now on. A `DB21M` is equipped with a `HUT` v3.1).  

<div figure-id="fig:HUT_layout">
     <img src="HUT_layout.jpg" style='width: 40em' />
</div>

Plug in the fan cable to the two pins as shown (note the orientation of the black and red cables!).

<div figure-id="fig:db21m-step_38">
     <img src="db21-step_38.png" style='width: 40em' />
</div>

### Step 39
Gently press the pin connector of the `HUT` on the pins on the NVIDIA Jetson Nano board. Make sure both motor cables are routed through the slit in the `HUT` board.

<div figure-id="fig:db21m-step_39">
     <img src="db21-step_39.png" style='width: 40em' />
</div>

### Step 40
Connect the `HUT` to the plate in the back using two nylon M2.5x10 screws (`B2`). Make sure the end of the 4-pin cable connected to the back bumper board is pointing to the right hand side.

<div figure-id="fig:db21m-step_40">
     <img src="db21-step_40.png" style='width: 40em' />
</div>

### Step 41
Connect the 4-pin cable of the back bumper board to the white connector in the back right corner of the `HUT`.

<div figure-id="fig:db21m-step_41">
     <img src="db21-step_41.png" style='width: 40em' />
</div>

### Step 42
Connect the first 6-pin motor cable of the left motor to the connector placed on the edge of the `HUT`.

<div figure-id="fig:db21m-step_42">
     <img src="db21-step_42.png" style='width: 40em' />
</div>

### Step 43
Connect the second 6-pin motor cable of the right motor to the other connector.

<div figure-id="fig:db21m-step_43">
     <img src="db21-step_43.png" style='width: 40em' />
</div>

### Step 44
Connect the 4-pin cable from the IMU to the corresponding plug shown in the picture.

<div figure-id="fig:db21m-step_44">
     <img src="db21-step_44.png" style='width: 40em' />
</div>

## Front Assembly {#howto-front-assembly-db21m}
The following steps 45 to 52 will guide you through the assembly of the camera unit as well as some more electronics and cables.

<div figure-id="fig:db21m-overview-step_45-52">
     <img src="db21-overview-step_45-52.png" style='width: 30em' />
</div>

### Step 45
If not already done, open the plug of the camera and push one side of the camera cable in. The orientation of the cable should be such that the copper pins on the camera cable face the camera plate. Then close the plug completely. Make sure to take off the plastic cap from the lens.

<div figure-id="fig:db21m-step_45">
     <img src="db21-step_45.png" style='width: 40em' />
</div>

### Step 46
Wire the camera cable through the 3D printed camera mount starting from the front. Then use 4 nylon M2x8 screws (`B1`) and 4 nylon M2 nuts (`N1`) on the other side to tighten the camera to the mount.

<div figure-id="fig:db21m-step_46">
     <img src="db21-step_46.png" style='width: 40em' />
</div>

### Step 47
Mount the camera part to the front bumper board using only one metal M3x8 screw (`B3`) and one metal M3 nut (`N3`).

<div figure-id="fig:db21m-step_47">
     <img src="db21-step_47.png" style='width: 40em' />
</div>

### Step 48
Mount the single nylon M3x5 stand-off (`S1`) with a metal M3x8 screw (`B3`) from the other side.

<div figure-id="fig:db21m-step_48">
     <img src="db21-step_48.png" style='width: 40em' />
</div>

### Step 49
Take one of the longest 4-pin cables and connect it to the front bumper board as shown.

<div figure-id="fig:db21m-step_49">
     <img src="db21-step_49.png" style='width: 40em' />
</div>

### Step 50
Wire the cable that you have just connected (step 49) through the upper plate and the connect it to the connector on the `HUT`, as shown below.

<div figure-id="fig:db21m-step_50">
     <img src="db21-step_50.png" style='width: 40em' />
</div>

### Step 51
Take the last of the long 4-pin cables and connect it to the front bumper board, similarly to step 49.

<div figure-id="fig:db21m-step_51">
     <img src="db21-step_51.png" style='width: 40em' />
</div>

### Step 52
Again, wire this cable through the upper plate and connect it to the `HUT`, similarly to step 50.

<div figure-id="fig:db21m-step_52">
     <img src="db21-step_52.png" style='width: 40em' />
</div>

### Step 53
Mount the front bumper board to the upper and lower plate using three metal M3x8 screws (`B3`). Make sure they are locked in place correctly.

<div figure-id="fig:db21m-step_53">
     <img src="db21-step_53.png" style='width: 40em' />
</div>

### Step 54
Open the camera slit on the NVIDIA Jetson Nano board by raising it on the sides (with care), and put in the other end of the camera cable.

Note: The orientation of the cable should be such that the blue part of the cable faces the camera (i.e., facing towards the front end of the Duckiebot).

<div figure-id="fig:db21m-step_54">
     <img src="db21-step_54.png" style='width: 40em' />
</div>

### Step 55
Attach the small blue distance sensor to the stand-off on the front bumper and tighten it with a nylon M3 nut (`N4`). Make sure to take off the small transparent cover from the sensor.

<div figure-id="fig:db21m-step_55">
     <img src="db21-step_55.png" style='width: 40em' />
</div>

### Step 56
Take the shortest 4-pin cable and connect the bottom of the time of flight sensor to the front bumper, as shown below.

<div figure-id="fig:db21m-step_56">
     <img src="db21-step_56.png" style='width: 40em' />
</div>


## Top Plate Assembly {#howto-interactive-cover-db21m}
The following steps 57 to 64 show the assembly of the top plate of the `DB21M`, containing a button and a screen.

<div figure-id="fig:db21m-overview-step_57-59">
     <img src="db21-overview-step_57-59.png" style='width: 30em' />
</div>

### Step 57
Remove the nut from the button, if necessary, and wire the button cables through the hole on the top plate  (marked as `03`).

Note: Mind the orientation; if the number is pointing downwards, we are good to go!

Once the button is pushed in completely, tighten it again with the flat nut.

<div figure-id="fig:db21m-step_57">
     <img src="db21-step_57.png" style='width: 40em' />
</div>

### Step 58
Mount the screen to the plate in a way the pins of the screen are pointing towards the button. Use 4 nylon M2.5 screws (`B2`) and 4 nylon M2.5 nuts (`N2`) for this.

<div figure-id="fig:db21m-step_58">
     <img src="db21-step_58.png" style='width: 40em' />
</div>

### Step 59
Have a look at the pin descriptions on the screen. Take the 4-pin cable with the long black connectors and connect the 4 loose ends to the screen.

Follow this pattern: GND-black, VCC-red, SCK-yellow, SDA-blue.

<div figure-id="fig:db21m-step_59">
     <img src="db21-step_59.png" style='width: 40em' />
</div>

### Step 60
Connect the end of the cable from the button to the connector on the `HUT`, as below.

<div figure-id="fig:db21m-step_60">
     <img src="db21-step_60.png" style='width: 40em' />
</div>

### Step 61
Connect the end of the cable from the screen to the 4 male pins on the `HUT` as shown. Check the colors of the cables so that the same goes to the same, i.e.: GND-black, 3.3V-red, SCL-yellow, SDA-blue.

<div figure-id="fig:db21m-step_61">
     <img src="db21-step_61.png" style='width: 40em' />
</div>

### Step 62
Gently place the cover plate on the chassis. Make sure the screws of the fan and the pins of the side plates are locked in place properly.

<div figure-id="fig:db21m-step_62">
     <img src="db21-step_62.png" style='width: 40em' />
</div>

### Step 63
Tighten the cover part using two nylon M2.5x10 screws (`B2`).

<div figure-id="fig:db21m-step_63">
     <img src="db21-step_63.png" style='width: 40em' />
</div>

### Step 64
Then, tighten the cover using two nylon M2.5 nuts (`N2`).

<div figure-id="fig:db21m-step_64">
     <img src="db21-step_64.png" style='width: 40em' />
</div>

## Power your Duckiebot {#howto-power-db21m}

In this step we will plug the various power cables to the `HUT`. One port will remain free. You can use this port to charge the Duckiebot.

<div figure-id="fig:db21m-how2charge">
     <img src="db21-how2charge.png" style='width: 30em' />
</div>


Warning: *Always* plug and unplug USB cables from the `HUT` with care!

### Step 65
Take the black USB cable that you have connected in step 29 and connect it to the micro USB connector on the `HUT` as shown.

<div figure-id="fig:db21m-step_65">
     <img src="db21-step_65.png" style='width: 40em' />
</div>

### Step 66
Similarly, connect the other USB cable (routed through the same hole) to the `HUT`.

<div figure-id="fig:db21m-step_66">
     <img src="db21-step_66.png" style='width: 40em' />
</div>

### Step 67
Finally, connect the last cable to the `HUT`.

<div figure-id="fig:db21m-step_67">
     <img src="db21-step_67.png" style='width: 40em' />
</div>

### Step 68
At that point, your Duckiebot is fully assembled!
For charging, connect the charging cable to the last free micro USB connector on the `HUT`. To avoid putting additional stress on the connector, you can leave this cable plugged in and store it somewhere under the blue top lid.

<div figure-id="fig:db21m-step_68A">
     <img src="db21-step_68A.png" style='width: 40em' />
</div>

Once your Duckiebot is fully charged, you can press the button of the battery on the side to power it up (do this ONLY if a flashed SD card has been inserted).

<div figure-id="fig:db21m-step_68B">
     <img src="db21-step_68B.png" style='width: 40em' />
</div>


## Additional Parts {#howto-additional-parts-db21m}

### Step 69
If you have an April tag take some glue and put it in between the two nylon screws on the top of you Duckiebot.

<div figure-id="fig:db21m-step_69">
     <img src="db21-step_69.png" style='width: 40em' />
</div>

### Step 70
If you have a circle pattern put it on the back plate of your Duckiebot.

<div figure-id="fig:db21m-step_70">
     <img src="db21-step_70.png" style='width: 40em' />
</div>        


## Check the outcome

<div class='requirements' markdown="1">

Checking: Look at the [Overview of interlocking parts](#fig:db21m-parts_indices) and make sure you have used each type at least once.

Checking: Have a look at all cable connectors and make sure they are plugged in completely.

Checking: check if all USB cables are plugged in completely. If you have already inserted a flashed SD card you are allowed to push the magic button on the battery.

</div>

## Troubleshooting {#op-faq-db21m}

<!--

Symptom: I can't find the blue chassis.

Resolution: It's *under* the white foam in the Duckiebox. Remove the inner packaging to access it.

-->

Symptom: Camera cable needs to be twisted to make the pins on the cable matching those in the connector. Is this normal?

Resolution: Yes this is normal. It might look a little nicer if you wire the camera cable around the metal stand-off next to the plug.

Symptom: The Duckiebattery does not fit flush in the compartment.

Resolution: Position it as it fits (at an angle). It will make the assembly a little trickier but everything will work out in the end.

Symptom: I don't have enough screws of a specific type.

Resolution: Each package has enough screws of each type, plus spares. It might happen to inadvertently using one type instead of the correct one, which will result in shortages towards the final stages. Following the instructions carefully will prevent this from happening.

Symptom: I can't screw the omni-directional wheel right; the screws don't fit all the way in the standoffs.

Resolution: Sometimes manufacturing inefficiencies make the thread inside the standoff shorter than it should. This happens only occasionally and it is not the norm. The solution is to orient, in case of need, the shorter threaded stand-off side towards above, on the side of the chassis.

Symptom: A piece broke while I was trying to assemble it!

Resolution: Mistakes happen. Some damages will not influence the functionality of the robot, others will be fixable at home with some tools, others could be showstoppers. Please take a picture of the damage and send an email to hardware@duckietown.com.

Symptom: The wheels tend to fall off the motors.

Resolution: You may remove the distance disks you put in step 22. But make sure that the wheels are still not touching the screws of the motor mounts.

Symptom: One of the black USB cables is too short to connect it to the HUT.

Resolution: The customized cables may undergo some manufacturing tolerances. If it does not fit, there is a second way to connect the cables. However, some minor functionalities might differ in that configuration (e.g. the fan might continue working when shutting down the NVIDIA Jetson Nano).

<div figure-id="fig:db21m-step_65-67">
     <img src="db21-step_65-67.jpg" style='width: 40em' />
</div>

Symptom: I don't understand what's going on with the connections

Resolution: This simplified block diagram of data and electrical connections of the `DB21M` might help:

<div figure-id="fig:db21m-block-diagram" figure-caption="`DB21M` block diagram of electrical and data connections.">
     <img src="db21-shcematics-block-diagram.png" style='width: 30em'/>
</div>

Symptom: I followed the instruction to the letter, but there is something off I can't quite put my finger on.

Resolution: You forgot to put a duckie on top of your Duckiebot!
