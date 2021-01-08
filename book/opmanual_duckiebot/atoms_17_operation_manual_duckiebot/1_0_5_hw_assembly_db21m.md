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
     <img src="Whats-in-the-box-ks-db18-v1.png" style='width: 37em' />
</div>

The assembly process is divided into 8 sub parts. They must be completed in the following order:

- [Part 1: Preliminary Steps](#howto-preliminary-db21m)
- [Part 2: Driving Units](#howto-driving-unit-db21m)
- [Part 3: Upper Frame](#howto-upper-frame-db21m)
- [Part 4: Computation Unit and Rear Assembly](#howto-comp-unit-db21m)
- [Part 5: Cable Management](#howto-cables-db21m)
- [Part 6: Front Assembly](#howto-front-assembly-db21m)
- [Part 7: Interactive Cover](#howto-interactive-cover-db21m)
- [Part 8: Optional Parts](#howto-optional-parts-db21m)

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
     <img src="db21-step_01.png" style='width: 37em' />
</div>

### Step 2
Check the hole pattern in the middle of the plate to make sure you are holding it the right way. Then insert two of the motor plates into the base plate.

<div figure-id="fig:db21-step_02">
     <img src="db21-step_02.png" style='width: 37em' />
</div>

### Step 3
Place one of the motors between the two plates and tighten it with two M3x30 screws (index *B5*) and two metal M3 nuts (index *N3*). It might be easier if you place the base plate on the table so the motor plates cannot fall out anymore.

<div figure-id="fig:db21-step_03">
     <img src="db21-step_03.png" style='width: 37em' />
</div>

### Step 4
Connect one of the 6 pin motor cables to the motor you have just mounted.

<div figure-id="fig:db21-step_04">
     <img src="db21-step_04.png" style='width: 37em' />
</div>

### Step 5
Connect the second 6 pin motor cable to the second motor.

<div figure-id="fig:db21-step_05">
     <img src="db21-step_05.png" style='width: 37em' />
</div>

### Step 6
Insert the other two motor plates to the slits in the base plate similar to step 2.

<div figure-id="fig:db21-step_06">
     <img src="db21-step_06.png" style='width: 37em' />
</div>

### Step 7
Tighten the second motor again with two M3x30 metal screws (index *B5*) and two metal M3 nuts (index *N3*) similar to what you have done in step 3.

<div figure-id="fig:db21-step_07">
     <img src="db21-step_07.png" style='width: 37em' />
</div>

### Step 8
Connect one of the three long 4 pin cables (colors: black-red-blue-yellow) to the white connector of the IMU (Inertia Measurement Unit) board.

<div figure-id="fig:db21-step_08">
     <img src="db21-step_08.png" style='width: 37em' />
</div>

### Step 9
Attach the IMU board to the base plate using two nylon M2.5x10 screws (index *B2*) and two nylon M2.5 nuts (index *N2*).

<div figure-id="fig:db21-step_09">
     <img src="db21-step_09.png" style='width: 37em' />
</div>

### Step 10
Turn the part around and make sure the three cables are routed through their corresponding holes or slits (Cable of the *left* motor through the *left* slit, cable of the *right* motor through the *right* slit).

<div figure-id="fig:db21-step_10">
     <img src="db21-step_10.png" style='width: 37em' />
</div>

### Step 11
Turn the assembly back again to the bottom side and mount the two M3x25 stand-offs (index *S3*) to the bottom plate with two metal M3x8 screws (index *B3*).

<div figure-id="fig:db21-step_11">
     <img src="db21-step_11.png" style='width: 37em' />
</div>

### Step 12
Mount the multi directional ("omni-") wheel to the stand-offs using again two of the metal M3x8 screws (index *B3*).

<div figure-id="fig:db21-step_12">
     <img src="db21-step_12.png" style='width: 37em' />
</div>



## Upper Frame {#howto-upper-frame-db21m}
The following steps 13 to 18 show the assembly of the lower body containing the special *Duckiebot* *Battery*:

<div figure-id="fig:db21-overview-step_13-18">
     <img src="db21-overview-step_13-18.png" style='width: 30em' />
</div>

### Step 13
Take the upper plate (number `01`) and  8 metal M3 nuts (index *N3*). Compare the hole in the green circles with the hole position on your plate and make sure they agree (if the number `01` is visible on top, you are good to go!).

<div figure-id="fig:db21-step_13">
     <img src="db21-step_13.png" style='width: 37em' />
</div>

### Step 14
Insert 4 nylon M2.5x10 screws (index *B2*) from the top and tighten them with 4 nylon M2.5 nuts (index *N2*).

<div figure-id="fig:db21-step_14">
     <img src="db21-step_14.png" style='width: 37em' />
</div>

### Step 15
Take the two plates with numbers `04L` and `04R` and insert three metal M3 nuts (index *N3*) into each of them.

<div figure-id="fig:db21-step_15">
     <img src="db21-step_15.png" style='width: 37em' />
</div>

### Step 16
Connect these two plates to the upper plate with a metal M3x8 screw (index *B3*) each. Note the slightly different holes in the side plates to mount them in the right way!

<div figure-id="fig:db21-step_16">
     <img src="db21-step_16.png" style='width: 37em' />
</div>

### Step 17
Insert the battery between the two side plates. Make sure the Duckietown sticker is on the bottom.

<div figure-id="fig:db21-step_17">
     <img src="db21-step_17.png" style='width: 37em' />
</div>

<div figure-id="fig:db21-step_17additional">
     <img src="db21-step_17additional.png" style='width: 20em' />
</div>

### Step 18
Take the two small locking plates carrying the number `07` and lock the battery in place.

<div figure-id="fig:db21-step_18">
     <img src="db21-step_18.png" style='width: 37em' />
</div>


## Computation Unit and Rear Assembly {#howto-comp-unit-db21m}
At that point, your Duckiebot is getting more and more in shape. The steps 19 to 27 will help you to complete the lower frame and mount the computation unit:

<div figure-id="fig:db21-overview-step_19-27">
     <img src="db21-overview-step_19-27.png" style='width: 30em' />
</div>

### Step 19
Take the part from steps 1 to 12 again and mount it directly to the assembly you have just created using 4 metal M3x8 screws. Make sure all the plates are locked together correctly.

<div figure-id="fig:db21-step_19">
     <img src="db21-step_19.png" style='width: 37em' />
</div>

### Step 20
Check the routing path of the two motor cables again from step 10 and wire them through the holes in the upper plate.

<div figure-id="fig:db21-step_20">
     <img src="db21-step_20.png" style='width: 37em' />
</div>

### Step 21
Do a similar procedure for the cable of the IMU unit.

<div figure-id="fig:db21-step_21">
     <img src="db21-step_21.png" style='width: 37em' />
</div>

### Step 22
Take the two yellow driving wheels and push them to the motors using one distance disk (index *S4*) between each of them.

<div figure-id="fig:db21-step_22">
     <img src="db21-step_22.png" style='width: 37em' />
</div>

### Step 23
Place the Jetson Nano on the 4 screws from step 14 but DO NOT tighten the Jetson with any nuts or stand-offs yet!

<div figure-id="fig:db21-step_23">
     <img src="db21-step_23.png" style='width: 37em' />
</div>


### Step 24
Take the side cover carrying the number `05L`. Insert a nylon M2.5 nut (index *N2*) and a metal M3 nut (index *N3*) into the plate (note the index engraving *N2* and *N3* on the plate itself). Then screw the plate to the frame using two metal M3x8 screws (index *B3*).
You may need to lift up the Jetson Nano a little bit in order to fit the side cover plate completely underneath the Jetson board.

<div figure-id="fig:db21-step_24">
     <img src="db21-step_24.png" style='width: 37em' />
</div>

### Step 25
Place the 4 black distance disks (index *S5*) on the nylon screws holding the Jetson Nano.

<div figure-id="fig:db21-step_25">
     <img src="db21-step_25.png" style='width: 37em' />
</div>

### Step 26
Tighten the Jetson Nano with the 6 brass stand-offs (index *S2*). Put two stand-offs each on the two screws in the front but only one each to the screws on the back side.

<div figure-id="fig:db21-step_26">
     <img src="db21-step_26.png" style='width: 37em' />
</div>

### Step 27
Take the side cover carrying the number `05R`. Insert a nylon M2.5 nut (index *N2*) and a metal M3 nut (index *N3*) into the plate (note the index engraving *N2* and *N3* on the plate itself) similar to step 24. Then screw the plate to the frame using two metal M3x8 screws (index *B3*).

<div figure-id="fig:db21-step_27">
     <img src="db21-step_27.png" style='width: 37em' />
</div>

## Cable Management {#howto-cables-db21m}

### Step 28




## Front Assembly {#howto-front-assembly-db21m}

## Interactive Cover {#howto-interactive-cover-db21m}

## Optional Parts {#howto-optional-parts-db21m}



## Check the outcome



## Visual inspection {#db21-visual-inspection}


## FAQ {#op-faq-db21}

Q: I found it hard to mount the omni-directional wheel / the back bumper / the Raspberry Pi because the holes weren't lining up.

A: Sometimes in life you have to push a little to make things happen. (But don't push too much or things will break!)

Q: I have a different color of the back patter plate, do I get the wrong one?

A: The color of this part can be of any and all are functionally equivalent.

Q: My battery is different from the one shown in the pictures! Did I get the wrong box?

A: If there is a duckie in or on your box, you most probably got the right one. We support different battery models. All supported models are functionally equivalent, although the form factor varies.
