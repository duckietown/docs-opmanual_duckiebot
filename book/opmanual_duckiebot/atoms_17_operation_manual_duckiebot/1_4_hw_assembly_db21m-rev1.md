# Assembly - Duckiebot `DB21M-Rev1` {#assembling-duckiebot-db21m-rev1 status=ready}

<!--

<div figure-id="fig:Duckiebook-Banner" figure-caption="The Duckiebot MOOC Founder's Edition, powered by NVIDIA Jetson Nano.">
     <img src="Duckiebook_Banner.png" style='width: 40em' />
</div>

-->

<div class='requirements' markdown="1">

Requires: Duckiebot `DB21M` parts ([get a `DB21M`](https://get.duckietown.com/)). If you are unsure what version of Duckiebot you have, check this overview of all existing [Duckiebot configurations](#duckiebot-configurations).

Requires: A micro SD card with the Duckiebot image on it. The procedure to flash the SD card is explained [here](#setup-duckiebot).

Requires: 3 hours of assembly time.

Result: An assembled Duckiebot in configuration `DB21M`.

</div>

<!--
-->
## Foreword

These instructions are your friend. Follow them carefully, especially if it's the first time you assemble a Duckiebot. Small variations might cause big effects (e.g., don't flip your cables!).

## Video tutorial {#db21m-rev1-assembly-video}

<div figure-id="fig:howto-assemble-db21m-rev1-video">
    <dtvideo src="vimeo:528621827"/>
</div>


## Overview

A Duckiebox contains the following components:

<div figure-id="fig:db21m-rev1-parts-overview">
     <img src="db21-rev1-parts-overview.jpg" style='width: 40em' />
</div>

The assembly process is divided in 9 parts. They must be completed in the following order:

- [Part 1: Preliminary Steps](#howto-preliminary-db21m-rev1)
- [Part 2: Drive Train](#howto-drive-train-db21m-rev1)
- [Part 3: Computation Unit](#howto-computation-db21m-rev1)
- [Part 4: Rear Assembly](#howto-rear-assembly-db21m-rev1)
- [Part 5: Front Assembly](#howto-front-assembly-db21m-rev1)
- [Part 6: Top Deck Assembly](#howto-top-deck-assembly-db21m-rev1)
- [FAQ section](#op-faq-db21m-rev1)

The FAQ section at the bottom of this page provides resolutions to common symptoms.

## Preliminary Steps {#howto-preliminary-db21m-rev1}

### Unboxing

Unbox all of your components and lay them out on a flat surface. Ensure that you have well lit, uncluttered space to work on.

Note: "The Duckiebox hides but does not steal". Your Duckiebot chassis is under the white protection foam. To get to the chassis, pull out the white foam from the box after removing everything. Mind that the upper part of the inside foam has several side pockets in addition to a main compartment where components are located.

Although not necessary, a small (M2.5) wrench might ease some of the next passages.

Note: we currently support the NVIDIA Jetson Nano 2GB developer kit, not 4GB.

### Plastic cover

Peel the plastic cover from all the chassis parts on both sides.

### Screws, Nuts and Stand-offs

Check the dimensions of each connecting part here before assembling it. This will prevent undesirable effects (e.g., nylon screws prevent electrical shorts).

<div figure-id="fig:db21m-rev1-parts-indices">
     <img src="db21-rev1-parts-indices.png" style='width: 40em' />
</div>


### Charge Battery via `HUT`
- Connect the battery and the HUT board as shown and make sure a green LED on the HUT is lit.
- Wait 30 minutes and then push the button on the battery.
- Check that the state of charge LEDs on the battery start blinking.
- Leave this setup until the battery is fully charged. This may take up to 5 hours.

<div figure-id="fig:db21m-rev1-step_00">
     <img src="db21-rev1-step_00.png" style='width: 40em' />
</div>



## Drive Train {#howto-drive-train-db21m-rev1}
In the following steps 1 to 16 you will build the *base plate* assembly.

<div figure-id="fig:db21m-rev1-overview-step_01-16">
     <img src="db21-rev1-overview-step_01-16.jpg" style='width: 60%' />
</div>

<div figure-id="fig:db21m-rev1-step_01">
     <img src="db21-rev1-step_01.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_02">
     <img src="db21-rev1-step_02.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_03">
     <img src="db21-rev1-step_03.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_04">
     <img src="db21-rev1-step_04.jpg" style='width: 100%' />
</div>

Note: Occasionally manufacturing tolerances (on the nut and the laser cut chassis) might prevent a flush fit. Trying a different nut or changing the insert direction might solve the problem.

<div figure-id="fig:db21m-rev1-step_05">
     <img src="db21-rev1-step_05.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_06">
     <img src="db21-rev1-step_06.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_07">
     <img src="db21-rev1-step_07.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_08">
     <img src="db21-rev1-step_08.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_09">
     <img src="db21-rev1-step_09.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_10">
     <img src="db21-rev1-step_10.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_11">
     <img src="db21-rev1-step_11.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_12">
     <img src="db21-rev1-step_12.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_13">
     <img src="db21-rev1-step_13.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_14">
     <img src="db21-rev1-step_14.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_15">
     <img src="db21-rev1-step_15.jpg" style='width: 100%' />
</div>

Remove the USB cable from the battery that you connected in one of the preliminary steps.

<div figure-id="fig:db21m-rev1-step_16">
     <img src="db21-rev1-step_16.jpg" style='width: 100%' />
</div>

Before proceeding, verify that no component is wiggling. The only things moving should be the cables and the sphere in the omni-wheel (and, yes, the motor axles). Proceed to gently tightening the screws of the offending parts, if necessary.

Congratulations, you just built the base plate Duckiebot assembly!

## Computation Unit {#howto-computation-db21m-rev1}
The following steps (17 to 25) guide through the *Computation* assembly:

<div figure-id="fig:db21m-rev1-overview-step_17-25">
     <img src="db21-rev1-overview-step_17-25.jpg" style='width: 60%' />
</div>

<!--
You can try to mount the wheels even without the distance disks. But make sure that the wheels do not touch the frame of the Duckiebot when turning.

<div figure-id="fig:db21m-step_22B">
     <img src="db21-step_22B.png" style='width: 40em' />
</div>

-->


<div figure-id="fig:db21m-rev1-step_17">
     <img src="db21-rev1-step_17.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_18">
     <img src="db21-rev1-step_18.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_19">
     <img src="db21-rev1-step_19.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_20">
     <img src="db21-rev1-step_20.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_21">
     <img src="db21-rev1-step_21.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_22">
     <img src="db21-rev1-step_22.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_23">
     <img src="db21-rev1-step_23.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_24">
     <img src="db21-rev1-step_24.jpg" style='width: 100%' />
</div>

Now connect it to the rest of the chassis assembled in steps 1 to 16. Make sure the laser cut parts are locked correctly.

<div figure-id="fig:db21m-rev1-step_25">
     <img src="db21-rev1-step_25.jpg" style='width: 100%' />
</div>


## Rear Assembly {#howto-rear-assembly-db21m-rev1}
At this point, we give more shape to the Duckiebot! The following steps 26 to 39 will help assemble the rear part and add most of the *cables*:

<div figure-id="fig:db21m-rev1-overview-step_26-39">
     <img src="db21-rev1-overview-step_26-39.jpg" style='width: 60%' />
</div>

<div figure-id="fig:db21m-rev1-step_26">
     <img src="db21-rev1-step_26.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_27">
     <img src="db21-rev1-step_27.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_28">
     <img src="db21-rev1-step_28.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_29">
     <img src="db21-rev1-step_29.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_30">
     <img src="db21-rev1-step_30.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_31">
     <img src="db21-rev1-step_31.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_32">
     <img src="db21-rev1-step_32.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_33">
     <img src="db21-rev1-step_33.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_34">
     <img src="db21-rev1-step_34.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_35">
     <img src="db21-rev1-step_35.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_36">
     <img src="db21-rev1-step_36.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_37">
     <img src="db21-rev1-step_37.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_38">
     <img src="db21-rev1-step_38.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_39">
     <img src="db21-rev1-step_39.jpg" style='width: 100%' />
</div>

## Front Assembly {#howto-front-assembly-db21m-rev1}

Steps 40 to 52 will guide you through the *front bumper* assembly:

<div figure-id="fig:db21m-rev1-overview-step_40-52">
     <img src="db21-rev1-overview-step_40-52.jpg" style='width: 60%' />
</div>

<div figure-id="fig:db21m-rev1-step_40">
     <img src="db21-rev1-step_40.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_41">
     <img src="db21-rev1-step_41.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_42">
     <img src="db21-rev1-step_42.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_43">
     <img src="db21-rev1-step_43.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_44">
     <img src="db21-rev1-step_44.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_45">
     <img src="db21-rev1-step_45.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_46">
     <img src="db21-rev1-step_46.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_47">
     <img src="db21-rev1-step_47.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_48">
     <img src="db21-rev1-step_48.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_49">
     <img src="db21-rev1-step_49.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_50">
     <img src="db21-rev1-step_50.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_51">
     <img src="db21-rev1-step_51.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_52">
     <img src="db21-rev1-step_52.jpg" style='width: 100%' />
</div>

## Top Deck Assembly {#howto-top-deck-assembly-db21m-rev1}
This last section with steps 53 to 63 will guide you through the assembly of the top deck:

<div figure-id="fig:db21m-rev1-overview-step_53-63">
     <img src="db21-rev1-overview-step_53-63.jpg" style='width: 60%' />
</div>

<div figure-id="fig:db21m-rev1-step_53">
     <img src="db21-rev1-step_53.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_54">
     <img src="db21-rev1-step_54.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_55">
     <img src="db21-rev1-step_55.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_56">
     <img src="db21-rev1-step_56.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_57">
     <img src="db21-rev1-step_57.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_58">
     <img src="db21-rev1-step_58.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_59">
     <img src="db21-rev1-step_59.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_60">
     <img src="db21-rev1-step_60.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_61">
     <img src="db21-rev1-step_61.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_62">
     <img src="db21-rev1-step_62.jpg" style='width: 100%' />
</div>

## Power your Duckiebot {#howto-power-db21m-rev1}

One of the USB ports on the HUT will remain free. You can use this port to charge your Duckiebot. To avoid putting additional stress on the connector, you can leave this cable plugged in and store it somewhere under the blue top lid.

<div figure-id="fig:db21m-rev1-how2charge">
     <img src="db21-rev1-how2charge.png" style='width: 60%' />
</div>

Warning: *Always* plug and unplug USB cables from the `HUT` with care!

Once your Duckiebot is fully charged, you can press the button of the battery on the side to power it up (do this ONLY if a flashed SD card has been inserted).

<div figure-id="fig:db21m-rev1-step_63">
     <img src="db21-rev1-step_63.jpg" style='width: 100%' />
</div>

Congratulations, your Duckiebot `DB21M` is completely assembled.

## Additional Parts {#howto-additional-parts-db21m-rev1}

### Back Pattern

<div figure-id="fig:db21m-rev1-step_64">
     <img src="db21-rev1-step_64.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_65">
     <img src="db21-rev1-step_65.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_66">
     <img src="db21-rev1-step_66.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_67">
     <img src="db21-rev1-step_67.jpg" style='width: 100%' />
</div>

<div figure-id="fig:db21m-rev1-step_68">
     <img src="db21-rev1-step_68.jpg" style='width: 100%' />
</div>

### April Tag

<div figure-id="fig:db21m-rev1-step_69">
     <img src="db21-rev1-step_69.jpg" style='width: 100%' />
</div>



## Check the outcome

* Look at the [Overview of interlocking parts](#fig:db21m-rev1-parts-indices) and make sure you have used each type at least once.

* Check all cable connectors and make sure they are plugged in completely. Do not use force on the Duckiebot, it is (almost) never useful and it might lead to undesirable outcomes.

* Make sure you have flashed your SD card with the latest version of the Duckiebot `DB21M` image.

Note: Version 1.2.2 is the minimum requirement for enabling battery code updates. Make sure you have at least this version (>22 March 2021).

* Make sure the SD card is inserted in Jetson Nano in the dedicated SD card slot under the main board. Do not plug it in the adapter and in a USB port. If you have already inserted a flashed SD card, you are allowed to push the magic button on the battery.



## Troubleshooting {#op-faq-db21m-rev1}

<!--

Symptom: I can't find the blue chassis.

Resolution: It's *under* the white foam in the Duckiebox. Remove the inner packaging to access it.

-->

Symptom: Camera cable needs to be twisted to make the pins on the cable matching those in the connector. Is this normal?

Resolution: Yes this is normal. It might look a little nicer if you wire the camera cable around the metal stand-off next to the plug.

Symptom: The Duckiebattery does not fit flush in the compartment.

Resolution: Position it as it fits (at an angle). It will make the assembly a little trickier but everything will work out in the end.

Symptom: I don't have enough screws of a specific type.

Resolution: Each package has enough screws of each type, plus spares of some. It might happen to inadvertently use one type instead of the correct one, which will result in shortages towards the final stages. Following the instructions carefully will prevent this from happening.

Symptom: I can't screw the omni-directional wheel right; the screws don't fit all the way in the standoffs.

Resolution: Sometimes manufacturing inefficiencies make the thread inside the standoff shorter than it should. This happens only occasionally and it is not the norm. The solution is to orient, in case of need, the shorter threaded stand-off side towards above, on the side of the chassis.

Symptom: A piece broke while I was trying to assemble it!

Resolution: Mistakes happen. Some damages will not influence the functionality of the robot, others will be fixable at home with some tools, others could be showstoppers. Please take a picture of the damage and send an email to hardware@duckietown.com.

Symptom: The wheels tend to fall off the motors.

Resolution: You may remove the distance disks you put in step 22. But make sure that the wheels are still not touching the screws of the motor mounts.

Symptom: My Duckiebot is driving backwards when pressing the key for straight forward.

Resolution: You have swapped the motor cables. Please check the steps 42 and 43 again and make sure you connected the cables the right way.

Symptom: I don't understand what's going on with the connections

Resolution: This simplified block diagram of data and electrical connections of the `DB21M` might help:

<div figure-id="fig:db21m-rev1-block-diagram" figure-caption="`DB21M` block diagram of electrical and data connections.">
     <img src="db21-rev1-schematics-block-diagram.png" style='width: 30em'/>
</div>

Symptom: I followed the instruction to the letter, but there is something off I can't quite put my finger on.

Resolution: You forgot to put a duckie on top of your Duckiebot!
