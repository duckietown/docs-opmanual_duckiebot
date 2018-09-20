# Preparing the power cable (only `DB17`) {#power-cable-prep-c0 status=ready}

In configuration `DB17` we will need a cable to power the DC motor HAT from the battery. The keen observer might have noticed that such a cable was not included in the [`DB17` Duckiebot parts](#acquiring-parts-c0) chapter. Here, we create this cable by splitting open any USB-A cable, identifying and stripping the power wires, and using them to power the DC motor HAT. If you are unsure about the definitions of the different Duckiebot configurations, read [](#duckiebot-configurations).

Note: It is important to note that these instructions are relevant only for assembling a `DB17-wjdc` configuration Duckiebot (or any subset of it). If you intend to build a `DB17-l` configuration Duckiebot, you can skip these instructions.

<div class='requirements' markdown="1">

Requires: One male USB-A to anything cable.

Requires: A pair of scissors.

Requires: A multimeter (only if you are not purchasing the [suggested components](#acquiring-parts-c0))

Requires: Time: 5 minutes

Results: One male USB-A to wires power cable

</div>

## Video tutorial

The following video shows how to prepare the USB power cable for the configuration `DB17`.

<div figure-id="fig:prepare-usb-cable">
    <dtvideo src="vimeo:236334476"/>
</div>


## Step-by-step guide

### Step 1: Find a cable

To begin with, find a male USB-A to anything cable.

If you have purchased the suggested components listed in [](#acquiring-parts-c0), you can use the longer USB cable contained inside the battery package ([](#figure:battery-pack-usb-cables)), which will be used as an example in these instructions.

<div figure-id="fig:battery-pack-usb-cables" figure-caption="The two USB cables in the suggested battery pack.">
     <img src="battery-pack-usb-cables.jpg" style='width: 15em'/>
</div>

Put the shorter cable back in the box, and open the longer cable ([](#figure:long-usb-cable))

<div figure-id="fig:long-usb-cable" figure-caption="Take the longer cable, and put the shorter on back in the box.">
     <img src="long-usb-cable.jpg" style='width: 15em'/>
</div>

### Step 2: Cut the cable

<div class="check" markdown="1">
Make sure the USB cable is _unplugged_ from any power source before proceeding.
</div>

 Take the scissors and cut it ([](#figure:cut-the-cable)) at the desired length from the USB-A port.

<div figure-id="fig:cut-the-cable" figure-caption="Cut the USB cable using the scissors.">
     <img src="cut-the-cable.jpg" style='width: 15em'/>
</div>

The cut will look like in [](#figure:the-cable-cut).

<div figure-id="fig:the-cable-cut" figure-caption="A cut USB cable.">
     <img src="the-cable-cut.jpg" style='width: 15em'/>
</div>

### Step 3: Strip the cable

Paying attention not to get hurt, strip the external white plastic. A way to do so without damaging the wires is shown in [](#figure:stripping-the-outside).

<div figure-id="fig:stripping-the-outside" figure-caption="Stripping the external layer of the USB cable.">
     <img src="stripping-the-outside.jpg" style='width: 15em'/>
</div>

After removing the external plastic, you will see four wires: black, green, white and red ([](#figure:under-the-hood)).

<div figure-id="fig:under-the-hood" figure-caption="Under the hood of a USB-A cable.">
     <img src="under-the-hood.jpg" style='width: 15em'/>
</div>

Once the bottom part of the external cable is removed, you will have isolated the four wires ([](#figure:stripped-the-outside)).

<div figure-id="fig:stripped-the-outside" figure-caption="The four wires inside a USB-A cable.">
     <img src="stripped-the-outside.jpg" style='width: 15em'/>
</div>

### Step 4: Strip the wires {#strip-the-power-wires}

<div class="check" markdown="1">
Make sure the USB cable is _unplugged_ from any power source before proceeding.
</div>

Once you have isolated the wires, strip them, and use the scissors to cut off the data wires (green and white, central positions) ([](#figure:strip-power-wires)).

<div figure-id="fig:strip-power-wires" figure-caption="Strip the power wires and cut the data wires.">
     <img src="strip-power-wires.jpg" style='width: 15em'/>
</div>

If you are not using the suggested cable, or want to verify which are the data and power wires, continue reading.

### Step 5: Find the power wires

If you are using the USB-A cable from the suggested battery pack, black and red are the power wires and green and white are instead for data.

If you are using a different USB cable, or are curious to verify that black and red actually are the power cables, take a multimeter and continue reading.

Plug the USB port inside a power source, e.g., the Duckiebot's battery. You can use some scotch tape to keep the cable from moving while probing the different pairs of wires with a multimeter. The voltage across the pair of power cables will be roughly twice the voltage between a power and data cable. The pair of data cables will have no voltage differential across them. If you are using the suggested Duckiebot battery as power source, you will measure around 5V across the power cables ([](#figure:testing-voltage-usb-cable-power-wires)).

<div figure-id="fig:testing-voltage-usb-cable-power-wires" figure-caption="Finding which two wires are for power.">
     <img src="testing-voltage-usb-cable-power-wires.jpg" style='width: 15em'/>
</div>

### Step 6: Test correct operation

You are now ready to secure the power wires to the DC motor HAT power pins. To do so though, you need to have soldered the boards first. If you have not done so yet, read [](#soldering-boards-c0).

If you have soldered the boards already, you may test correct functionality of the newly crafted cable. Connect the battery with the DC motor HAT by making sure you plug the black wire in the pin labeled with a minus: `-` and the red wire to the plus: `+` ([](#figure:final-result-power-c0)).

<div figure-id="fig:final-result-power-c0" figure-caption="Connect the power wires to the DC motor HAT">
     <img src="final-result-power-c0.jpg" style='width: 15em'/>
</div>
