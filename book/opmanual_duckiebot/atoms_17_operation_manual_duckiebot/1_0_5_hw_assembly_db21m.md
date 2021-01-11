# Assemblying the Duckiebot `DB-MOOC` {#assembling-duckiebot-db21 status=ready}

<div figure-id="fig:Duckiebook-Banner" figure-caption="Components in the Duckiebox.">
     <img src="Duckiebook_Banner.png" style='width: 40em' />
</div>

Note: This section is still under construction!!!

Assembly instructions for the configuration of the Duckiebot for the MOOC.


<div class='requirements' markdown="1">

Requires: Duckiebot `DB-MOOC` parts. Here is an [overview of all existing configurations](#duckiebot-configurations).

Requires: A microSD card with the Duckiebot image on it. The procedure to flash the SD card is explained [here](#setup-duckiebot).

Requires: 2 hours of assembly time.

Result: An assembled Duckiebot in configuration `DB-MOOC`.

</div>

Overview of all parts in your Duckiebox:
TODO: make overview picture of the DB21 Parts

<div figure-id="fig:duckiebot-components-db21m" figure-caption="Components in the Duckiebox.">
     <img src="Whats-in-the-box-ks-db18-v1.png" style='width: 40em' />
</div>

The assembly process is divided into 9 sub parts. They must be completed in the following order:

- [Part 1: Preliminary Steps](#howto-preliminary-db21m)
- [Part 2: Driving Units](#howto-driving-unit-db21m)
- [Part 3: Upper Frame](#howto-upper-frame-db21m)
- [Part 4: Computation Unit and Rear Assembly](#howto-comp-unit-db21m)
- [Part 5: Cable Management](#howto-cables-db21m)
- [Part 6: Front Assembly](#howto-front-assembly-db21m)
- [Part 7: Interactive Cover](#howto-interactive-cover-db21m)
- [Part 8: Power your Duckiebot](#howto-power-db21m)
- [Part 9: Optional Parts](#howto-optional-parts-db21m)

## Preliminary Steps {#howto-preliminary-db21m}

### Unboxing
Unbox all of your components and lay them out on a flat surface. Ensure that you have well lit, uncluttered space to work on.

Although not necessary, you might find useful getting a small (M2.5) wrench to ease some of the passages.

### Plastic cover
Peel the plastic cover from all the chassis parts (except the bumper bracers) on both sides.

TODO: add picture for removing plastic cover

### Screws, Nuts and Stand-offs
Each type of screw, nut and stand-off has its own number. You will find the corresponding number on the picture of each step. Make sure to always use the right parts (e.g. nylon screws prevent electrical short circuits). Otherwise, your Duckiebot may not be fully functional.

Remember that instructions are your friend! At least at this stage, try to follow them precisely. The [FAQ](#op-faq-db21) section at the bottom of this page might be the first thing to visit if something is not clear. This may already answer your comments, questions or doubts.


If regardless of this foreword you still choose to try and figure it out yourself, it's ok, but for the love of what is precious, *do not plug the battery in until you have performed a visual inspection* [here](#db21-visual-inspection).  


## Driving Units {#howto-driving-unit-db21m}
From the Duckiebox package take the following components:

TODO: add picture for part overview of steps 1-12

In the following steps 1 to 12 you will build this first part of your Duckiebot `DB-MOOC`:

<div figure-id="fig:db21-overview-step_01-12">
     <img src="db21-overview-step_01-12.png" style='width: 30em' />
</div>


### Step 1
Take the base plate (number `02` on the bottom side) and insert two metal M3 nuts as shown.

<div figure-id="fig:db21-step_01">
     <img src="db21-step_01.png" style='width: 40em' />
</div>

### Step 2
Check the hole pattern in the middle of the plate to make sure you are holding it the right way. Then insert two of the motor plates into the base plate.

<div figure-id="fig:db21-step_02">
     <img src="db21-step_02.png" style='width: 40em' />
</div>

### Step 3
Place one of the motors between the two plates and tighten it with two M3x30 screws (index *B5*) and two metal M3 nuts (index *N3*). It might be easier if you place the base plate on the table so the motor plates cannot fall out anymore.

<div figure-id="fig:db21-step_03">
     <img src="db21-step_03.png" style='width: 40em' />
</div>

### Step 4
Connect the first 6-pin motor cable (length = 220mm) to the motor you have just mounted.

<div figure-id="fig:db21-step_04">
     <img src="db21-step_04.png" style='width: 40em' />
</div>

### Step 5
Connect the second 6-pin motor cable (length = 220mm) to the second motor.

<div figure-id="fig:db21-step_05">
     <img src="db21-step_05.png" style='width: 40em' />
</div>

### Step 6
Insert the other two motor plates to the slits in the base plate similar to step 2.

<div figure-id="fig:db21-step_06">
     <img src="db21-step_06.png" style='width: 40em' />
</div>

### Step 7
Tighten the second motor again with two M3x30 metal screws (index *B5*) and two metal M3 nuts (index *N3*) similar to what you have done in step 3.

<div figure-id="fig:db21-step_07">
     <img src="db21-step_07.png" style='width: 40em' />
</div>

### Step 8
Connect one of the three long 4-pin cables (length = 260mm) to the white connector of the IMU (Inertia Measurement Unit) board.

<div figure-id="fig:db21-step_08">
     <img src="db21-step_08.png" style='width: 40em' />
</div>

### Step 9
Attach the IMU board to the base plate using two nylon M2.5x10 screws (index *B2*) and two nylon M2.5 nuts (index *N2*).

<div figure-id="fig:db21-step_09">
     <img src="db21-step_09.png" style='width: 40em' />
</div>

### Step 10
Turn the part around and make sure the three cables are routed through their corresponding holes or slits (Cable of the *left* motor through the *left* slit, cable of the *right* motor through the *right* slit).

<div figure-id="fig:db21-step_10">
     <img src="db21-step_10.png" style='width: 40em' />
</div>

### Step 11
Turn the assembly back again to the bottom side and mount the two M3x25 stand-offs (index *S3*) to the bottom plate with two metal M3x8 screws (index *B3*).

<div figure-id="fig:db21-step_11">
     <img src="db21-step_11.png" style='width: 40em' />
</div>

### Step 12
Mount the multi directional ("omni-") wheel to the stand-offs using again two of the metal M3x8 screws (index *B3*).

<div figure-id="fig:db21-step_12">
     <img src="db21-step_12.png" style='width: 40em' />
</div>



## Upper Frame {#howto-upper-frame-db21m}
The following steps 13 to 18 show the assembly of the lower body containing the special *Duckiebot* *Battery*:

<div figure-id="fig:db21-overview-step_13-18">
     <img src="db21-overview-step_13-18.png" style='width: 30em' />
</div>

### Step 13
Take the upper plate (number `01`) and  8 metal M3 nuts (index *N3*). Compare the hole in the green circles with the hole position on your plate and make sure they agree (if the number `01` is visible on top, you are good to go!).

<div figure-id="fig:db21-step_13">
     <img src="db21-step_13.png" style='width: 40em' />
</div>

### Step 14
Insert 4 nylon M2.5x10 screws (index *B2*) from the top and tighten them with 4 nylon M2.5 nuts (index *N2*).

<div figure-id="fig:db21-step_14">
     <img src="db21-step_14.png" style='width: 40em' />
</div>

### Step 15
Take the two plates with numbers `04L` and `04R` and insert three metal M3 nuts (index *N3*) into each of them.

<div figure-id="fig:db21-step_15">
     <img src="db21-step_15.png" style='width: 40em' />
</div>

### Step 16
Connect these two plates to the upper plate with a metal M3x8 screw (index *B3*) each. Note the slightly different holes in the side plates to mount them in the right way!

<div figure-id="fig:db21-step_16">
     <img src="db21-step_16.png" style='width: 40em' />
</div>

### Step 17
Insert the battery between the two side plates. Make sure the Duckietown sticker is on the bottom.

<div figure-id="fig:db21-step_17">
     <img src="db21-step_17.png" style='width: 40em' />
</div>

<div figure-id="fig:db21-step_17additional">
     <img src="db21-step_17additional.png" style='width: 20em' />
</div>

### Step 18
Take the two small plates carrying the number `07` and lock the battery in place.

<div figure-id="fig:db21-step_18">
     <img src="db21-step_18.png" style='width: 40em' />
</div>


## Computation Unit and Rear Assembly {#howto-comp-unit-db21m}
At that point, your Duckiebot is getting more and more in shape. The steps 19 to 27 will help you to complete the lower frame and mount the computation unit:

<div figure-id="fig:db21-overview-step_19-27">
     <img src="db21-overview-step_19-27.png" style='width: 30em' />
</div>

### Step 19
Take the part from steps 1 to 12 again and mount it directly to the assembly you have just created using 4 metal M3x8 screws. Make sure all the plates are locked in place correctly.

<div figure-id="fig:db21-step_19">
     <img src="db21-step_19.png" style='width: 40em' />
</div>

### Step 20
Check the routing path of the two motor cables again from step 10 and wire them through the holes in the upper plate.

<div figure-id="fig:db21-step_20">
     <img src="db21-step_20.png" style='width: 40em' />
</div>

### Step 21
Do a similar procedure for the cable of the IMU unit.

<div figure-id="fig:db21-step_21">
     <img src="db21-step_21.png" style='width: 40em' />
</div>

### Step 22
Take the two yellow driving wheels and push them to the motors using one distance disk (index *S4*) between each of them.

<div figure-id="fig:db21-step_22">
     <img src="db21-step_22.png" style='width: 40em' />
</div>

### Step 23
Place the Jetson Nano on the 4 screws from step 14 but DO NOT tighten the Jetson with any nuts or stand-offs yet!

<div figure-id="fig:db21-step_23">
     <img src="db21-step_23.png" style='width: 40em' />
</div>


### Step 24
Take the side cover carrying the number `05L`. Insert a nylon M2.5 nut (index *N2*) and a metal M3 nut (index *N3*) into the plate (note the index engraving *N2* and *N3* on the plate itself). Then screw the plate to the frame using two metal M3x8 screws (index *B3*).
You may need to lift up the Jetson Nano a little bit in order to fit the side cover plate completely underneath the Jetson board.

<div figure-id="fig:db21-step_24">
     <img src="db21-step_24.png" style='width: 40em' />
</div>

### Step 25
Place the 4 black distance disks (index *S5*) on the nylon screws holding the Jetson Nano.

<div figure-id="fig:db21-step_25">
     <img src="db21-step_25.png" style='width: 40em' />
</div>

### Step 26
Tighten the Jetson Nano with the 6 brass stand-offs (index *S2*). Put two stand-offs each on the two screws in the front but only one each to the screws on the back side.

<div figure-id="fig:db21-step_26">
     <img src="db21-step_26.png" style='width: 40em' />
</div>

### Step 27
Take the side cover carrying the number `05R`. Insert a nylon M2.5 nut (index *N2*) and a metal M3 nut (index *N3*) into the plate (note the index engraving *N2* and *N3* on the plate itself) similar to step 24. Then screw the plate to the frame using two metal M3x8 screws (index *B3*).

<div figure-id="fig:db21-step_27">
     <img src="db21-step_27.png" style='width: 40em' />
</div>

## Cable Management {#howto-cables-db21m}

### Step 28
Take the first black USB cable that contains three connectors. Plug in the big USB-A connector that combines the two cables into the left plug of the battery. Then, connect the other USB-A to the lower port of the USB stack on the Jetson Nano. The micro USB connect must not be connected at that stage!

<div figure-id="fig:db21-step_28">
     <img src="db21-step_28.png" style='width: 40em' />
</div>

### Step 29
Take the second black USB cable with a micro USB plug at each end and connect the angled connector to the middle port on the battery. Wire the cable through the chassis as shown. The other micro USB connector must be left free at that stage!

Note: You need to mark the free end of this cable somehow (e.g. with a sticker or tape) as we need to distinguish it at the end of the assembly process from the other cables!

<div figure-id="fig:db21-step_29">
     <img src="db21-step_29.png" style='width: 40em' />
</div>

### Step 30
Take the third black USB cable with a micro USB plug on one side and an angled USB-A plug on the other side. Connect the USB-A plug to the right port on the battery. Again, wire the cable through the chassis as shown and leave the micro USB connector free on the other side!

<div figure-id="fig:db21-step_30">
     <img src="db21-step_30.png" style='width: 40em' />
</div>

### Step 31
Connect the WIFI dongle to the upper USB-A port on the USB stack on the Jetson Nano.

<div figure-id="fig:db21-step_31">
     <img src="db21-step_31.png" style='width: 40em' />
</div>

### Step 32
Take the back bumper board and connect the 4-pin cable (length = 120mm) to the white plug on the board.

<div figure-id="fig:db21-step_32">
     <img src="db21-step_32.png" style='width: 40em' />
</div>

### Step 33
Wire the cable attached to the back bumper through the same hole in the upper plate as the motor cable of the left driving motor.

<div figure-id="fig:db21-step_33">
     <img src="db21-step_33.png" style='width: 40em' />
</div>

### Step 34
When attaching the back bumper board to the chassis, make sure the pins of the lower and upper plate all fit well. Tighten the board with three metal screws (index *B3*).

<div figure-id="fig:db21-step_34">
     <img src="db21-step_34.png" style='width: 40em' />
</div>

### Step 35
Take the plate carrying the number `06` and press two nylon M2.5 nuts (index *N2*) into the corresponding slits.

<div figure-id="fig:db21-step_35">
     <img src="db21-step_35.png" style='width: 40em' />
</div>

### Step 36
Mount the plate number `06` to the back of the chassis using two metal M3x8 screws (index *B3*). Note the orientation of the plate: if the number `06` is pointing towards the robot, you are good to go!

<div figure-id="fig:db21-step_36">
     <img src="db21-step_36.png" style='width: 40em' />
</div>

### Step 37
Take the fan and mount it on top of the heat sink of the Jetson Nano using 4 metal M3x12 screws (index *B4*). Make sure the cable of the fan is pointing to the back right side (maybe you need to need to push a little more for those screws as the thread has to cut it's way through the heat sink at the first time).

<div figure-id="fig:db21-step_37">
     <img src="db21-step_37.png" style='width: 40em' />
</div>

### Step 38
Take the electronics board (further called *HUT*) and plug in the fan cable to the two pins as shown (note the orientation of the black and red cables!).

<div figure-id="fig:db21-step_38">
     <img src="db21-step_38.png" style='width: 20em' />
</div>

### Step 39
Gently press the pin connector of the HUT to the pins on the Jetson Nano and tighten it with two nylon M2.5 nuts (index *N2*). Make sure both motor cables are routed through the slit in the HUT board.

<div figure-id="fig:db21-step_39">
     <img src="db21-step_39.png" style='width: 40em' />
</div>

### Step 40
Connect the HUT to the plate in the back using two nylon M2.5x10 screws (index *B2*). Make sure the end of the 4-pin cable connected to the back bumper board is pointing to the right hand side.

<div figure-id="fig:db21-step_40">
     <img src="db21-step_40.png" style='width: 40em' />
</div>

### Step 41
Connect the 4-pin cable of the back bumper board to the white connector in the back right corner of the HUT.

<div figure-id="fig:db21-step_41">
     <img src="db21-step_41.png" style='width: 40em' />
</div>

### Step 42
Connect the first 6-pin motor cable of the left motor to the connector placed on the edge of the HUT.

<div figure-id="fig:db21-step_42">
     <img src="db21-step_42.png" style='width: 40em' />
</div>

### Step 43
Connect the second 6-pin motor cable of the right motor to the other connector.

<div figure-id="fig:db21-step_43">
     <img src="db21-step_43.png" style='width: 40em' />
</div>

### Step 44
Connect the 4-pin cable from the IMU to the corresponding plug shown in the picture.

<div figure-id="fig:db21-step_44">
     <img src="db21-step_44.png" style='width: 40em' />
</div>

## Front Assembly {#howto-front-assembly-db21m}
The following steps 45 to 52 will guide you through the assembly of the camera unit as well as some more electronics and cables.

<div figure-id="fig:db21-overview-step_45-52">
     <img src="db21-overview-step_45-52.png" style='width: 30em' />
</div>

### Step 45
Open the plug of the camera and push one side of the camera (length = 100mm) into the plug. Make sure that the blue are is facing the direction of the camera lens and then close the plug completely.

<div figure-id="fig:db21-step_45">
     <img src="db21-step_45.png" style='width: 40em' />
</div>

### Step 46
Wire the camera cable through the 3D printed camera mount starting from the front. Then use 4 nylon M2x8 screws (index *B1*) and 4 nylon M2 nuts (index *N1*) on the other side to tighten the camera to the mount.

<div figure-id="fig:db21-step_46">
     <img src="db21-step_46.png" style='width: 40em' />
</div>

### Step 47
Mount the camera part to the front bumper board using only one metal M3x8 screw (index *B3*) and one metal M3 nut (index *N3*).

<div figure-id="fig:db21-step_47">
     <img src="db21-step_47.png" style='width: 40em' />
</div>

### Step 48
Mount the single nylon M3x5 stand-off (index *S1*) with a metal M3x8 screw (index *B3*) from the other side.

<div figure-id="fig:db21-step_48">
     <img src="db21-step_48.png" style='width: 40em' />
</div>

### Step 49
Take the second 4-pin cable (length = 260mm) and connect it to the front bumper board as shown.

<div figure-id="fig:db21-step_49">
     <img src="db21-step_49.png" style='width: 40em' />
</div>

### Step 50
Wire the cable that you have just connected to the front bumper in step 49 through the upper plate and the connect it to the connector on the HUT as shown.

<div figure-id="fig:db21-step_50">
     <img src="db21-step_50.png" style='width: 40em' />
</div>

### Step 51
Take the third 4-pin cable (length = 260mm) and connect it to the front bumper board similar to step 49.

<div figure-id="fig:db21-step_51">
     <img src="db21-step_51.png" style='width: 40em' />
</div>

### Step 52
Again, wire this cable through the upperplate and connect it to the HUT similar to what you have done in step 50.

<div figure-id="fig:db21-step_52">
     <img src="db21-step_52.png" style='width: 40em' />
</div>

### Step 53
Mount the front bumper board to the upper and lower plate and make sure they are locked correctly in the board.

<div figure-id="fig:db21-step_53">
     <img src="db21-step_53.png" style='width: 40em' />
</div>

### Step 54
Open the camera plug on the Jetson Nano and put in the other end of the camera cable. Note the orientation of the cable: the pins on the cable have to face the Jetson (towards the back of the robot).

<div figure-id="fig:db21-step_54">
     <img src="db21-step_54.png" style='width: 40em' />
</div>

### Step 55
Attach the small blue distance sensor to the stand-off on the front bumper and tighten it with a nylon M3 nut (index *N4*).

<div figure-id="fig:db21-step_55">
     <img src="db21-step_55.png" style='width: 40em' />
</div>

### Step 56
Take the shortest 4-pin cable (length = 50mm) and connect one end to the plug on the bottom of the distance sensor. Plug in the other side to the connector on the front bumper as shown.

<div figure-id="fig:db21-step_56">
     <img src="db21-step_56.png" style='width: 40em' />
</div>


## Interactive Cover {#howto-interactive-cover-db21m}
The following steps 57 to 64 show the assembly of the interactive top of the `DB-MOOC` containing a button and a screen.

<div figure-id="fig:db21-overview-step_57-59">
     <img src="db21-overview-step_57-59.png" style='width: 30em' />
</div>

### Step 57
Remove the nut from the button if necessary and wire the cable of the button through the hole on the plate carrying the number `03`. Note the orientation of this plate: if the number is pointing downwards, you are good to go! Once the button is pushed in completely, tighten it again with the flat nut.

<div figure-id="fig:db21-step_57">
     <img src="db21-step_57.png" style='width: 40em' />
</div>

### Step 58
Mount the screen to the plate in a way the pins of the screen are pointing towards the button. Use 4 nylon M2.5 screws (index *B2*) and 4 nylon M2.5 nuts (index *N2*) for this.

<div figure-id="fig:db21-step_58">
     <img src="db21-step_58.png" style='width: 40em' />
</div>

### Step 59
Have a look at the pin description on the screen. Take the 4-pin cable with the long black connectors and connect the 4 loose ends to the screen. Make sure that the colors match the ones in the picture, namely: GND-black, VCC-red, SCL-yellow, SCD-blue.

<div figure-id="fig:db21-step_59">
     <img src="db21-step_59.png" style='width: 40em' />
</div>

### Step 60
Connect the end of the cable from the button to the connector on the HUT as shown.

<div figure-id="fig:db21-step_60">
     <img src="db21-step_60.png" style='width: 40em' />
</div>

### Step 61
Connect the end of the cable from the screen to the 4 male pins on the HUT as shown. Check the colors of the cables and make sure they are identical to the ones on the picture.

<div figure-id="fig:db21-step_61">
     <img src="db21-step_61.png" style='width: 40em' />
</div>

### Step 62
Gently place the cover plate on the chassis. Make sure the screws of the fan and the pins of the side plates are locked in place properly.

<div figure-id="fig:db21-step_62">
     <img src="db21-step_62.png" style='width: 40em' />
</div>

### Step 63
Tighten the cover part using two nylon M2.5x10 screws (index *B2*).

<div figure-id="fig:db21-step_63">
     <img src="db21-step_63.png" style='width: 40em' />
</div>

### Step 64
Further, tighten the cover part using two nylon M2.5 nuts (index *N2*).

<div figure-id="fig:db21-step_64">
     <img src="db21-step_64.png" style='width: 40em' />
</div>

## Power your Duckiebot {#howto-power-db21m}

### Step 65
Take the black USB cable that you have marked in step 29 and connect it to the second micro USB connector on the HUT counting from the back.

<div figure-id="fig:db21-step_65">
     <img src="db21-step_65.png" style='width: 40em' />
</div>

### Step 66
Take the black USB cable that is routed through the same hole as the one you have just connected in the previous step 65. Connect it to the back micro USB connector on the HUT as shown.

<div figure-id="fig:db21-step_66">
     <img src="db21-step_66.png" style='width: 40em' />
</div>

### Step 67
Finally, take the last black USB cable that has not been connected yet. Connect it to the front micro USB connector on the HUT.

<div figure-id="fig:db21-step_67">
     <img src="db21-step_67.png" style='width: 40em' />
</div>

### Step 68
At that point, your Duckiebot is fully assembled! To power it on, push the button of the battery on the side of the robot.

<div figure-id="fig:db21-step_68">
     <img src="db21-step_68.png" style='width: 40em' />
</div>


## Optional Parts {#howto-optional-parts-db21m}

### Step 69

### Step 70


## Check the outcome

TODO: define steps for checking

## Visual inspection {#db21-visual-inspection}

TODO: define steps for visual inspection

## FAQ {#op-faq-db21}

Q: CAMERA CABLE: I had to twist the cable of the camera by 180 degrees in order to make the pins on the cable matching the pins in the connector (facing the back side of the robot). Is this normal?

A: Yes this is normal! It might look a little nicer if you wire the camera cable around the metal stand-off next to the plug.
