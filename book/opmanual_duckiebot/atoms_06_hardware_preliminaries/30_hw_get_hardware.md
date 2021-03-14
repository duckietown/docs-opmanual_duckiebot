# Getting the Duckiebot hardware {#get-db-hw status=ready}

<div class='requirements' markdown="1">

Requires: Knowledge of [Duckiebot hardware configurations](#duckiebot-configurations)

Results: Parts to assemble a Duckiebot.

</div>

## Foreword

You can acquire Duckiebots in two ways.

* **"One click" solution**: All Duckiebot configurations include some hardware components that are custom made. Therefore, the easiest way is to order a complete set directly by contacting the [Duckietown project](https://www.duckietown.org/about/hardware).

* **Do it yourself**: Some individual parts are made of components that you can source independently: see next section.

## Acquiring the parts individually {#db-opmanual-get-parts status=ready}

In general, keep in mind that:

- The links might expire, or the prices might vary.
- Shipping times and fees vary, and are not included in the prices shown below.
- International deliveries are subject to additional custom clearances and import fees.
- Substitutions are OK for the mechanical components,
  and not OK for all the electronics, unless you are OK in writing
  some software. Limited technical support will be offered for hardware customizations.
- Buying the parts for more than one Duckiebot makes each one cheaper than buying only one.
- For some components, the links we provide contain more bits than actually needed.

Here, we provide a link to all bits and pieces that can be sourced independently, along with the price tag.

<div markdown="1">

 <col2 id='materials' figure-id="tab:materials" figure-caption="Bill of materials">
    <s>[Chassis for DB18 or DB19](http://www.kr4.us/magician-chassis-rob-12866.html)</s>        <s>USD 20</s>
    <s>[Camera with 160-FOV Fisheye Lens](https://tinyurl.com/ybwrcywc)</s>                         <s>USD 39</s>
    <s>[Camera Mount](https://tinyurl.com/ybyewdrt)</s>                         <s>USD 4</s>
    <s>[300mm Camera Cable](https://www.adafruit.com/product/1648)</s>                         <s>USD 2</s>
    <s>[Raspberry Pi 3 - Model B+](http://tinyurl.com/y66e43ks)</s>                         <s>USD 39</s>
    <s>[Heat Sinks](https://tinyurl.com/yanradnp)</s>                         <s>USD 3</s>
    <s>[Power supply for Raspberry Pi](https://www.adafruit.com/product/1995)</s>                         <s>USD 7.50</s>
    <s>[16 GB Class 10 MicroSD Card](http://tinyurl.com/y2dlbnfs)</s>                         <s>USD 10</s>
    <s>[Mirco SD card reader](https://www.adafruit.com/product/939)</s><s>USD 6 </s>
    <s>[16 Nylon Standoffs (M2.5 12mm F 6mm M)](https://tinyurl.com/y9uy73b2)</s>                         <s>USD 0.06/piece</s>
    <s>[4 Nylon Hex Nuts (M2.5)](https://tinyurl.com/ydy4znem)</s>                      <s>USD 0.02/piece</s>
    <s>[4 Nylon Screws (M2.5x10)](https://tinyurl.com/ya2uo9so)</s>                     <s>USD 0.05/piece</s>
    <s>[2 Zip Ties (300x5mm)](https://tinyurl.com/yb8v3nns)</s>                         <s>USD 9</s>
    <s>[Wireless Adapter (5 GHz)](https://tinyurl.com/ycvu7ok3)</s>           <s>USD 25</s>
    <s>[Tiny 32GB USB Flash Drive](http://tinyurl.com/y4smbqe8)</s>                                    <s>USD 10</s>
    <s>[Power Cable](https://tinyurl.com/yaptpssu) (`DB17-l1`)</s> <s>USD 7.80</s>
    <s>[Male-Male Jumper Wire](https://www.adafruit.com/products/1957) (`DB17-l1`)</s> <s>USD 1.95</s>
    <s>[8 M3x10 pan head screws](https://www.mcmaster.com/#92005a120/=19lvrzk) (`DB17-l2`)</s><s>USD 7</s>
   <s>[8 M3 nuts](https://www.mcmaster.com/#90591a250/=19lvsom) (`DB17-l2`)</s> <s>USD 7</s>
   <s>Bumpers set (`DB17-l2`)</s>  <s>USD 7 (custom made)</s>
   <s>Bumper bracers set (`DB17-l2`)</s>  <s>USD 7 (custom made)</s>
   <s>[LEDs](https://www.adafruit.com/product/848) (`DB17-l3`)</s> <s>USD 10</s>
   <s>[LED HAT](https://tinyurl.com/ydh9wqp5) (`DB17-l3`)</s> <s>USD 9/piece (but 3 pieces minimum)</s>
   <s>[20 Female-Female Jumper Wires (300mm)](https://www.adafruit.com/products/793) (`DB17-l3`)</s> <s>USD 8</s>
   <s>[4 4 pin female header](http://www.digikey.com/product-detail/en/PPTC041LFBN-RC/S7002-ND/810144) (`DB17-l3`)</s> <s>USD 0.60/piece</s>
   <s>[12 pin male header](http://www.digikey.com/product-detail/en/amphenol-fci/68000-412HLF/609-3266-ND/1878525) (`DB17-l3`)</s> <s>USD 0.48/piece</s>
   <s>[2 16 pin male
   header](http://www.digikey.com/product-detail/en/0022284160/WM50014-16-ND/313801) (`DB17-l3`)</s> <s>USD 0.61/piece</s>
   <s>[3 pin male header](http://www.digikey.com/product-detail/en/M20-9990345/952-2263-ND/3728227) (`DB17-l3`)</s> <s>USD 0.10/piece</s>
   <s>[2 pin female shunt jumper](http://www.digikey.com/product-detail/en/382811-8/A26228-ND/293121) (`DB17-l3`)</s> <s>USD 2/piece</s>
   <s>[40 pin female header](https://www.adafruit.com/products/2222) (`DB17-l3`)</s> <s>USD 1.50</s>
   <s>[Soldering tools](http://tinyurl.com/yyusy73b)</s> <s>USD 20</s>
 </col2>

</div>

<style>
#materials {
    font-size: 80%;
}
#materials TD {
    text-align: left;
}
</style>

## Chassis

We selected the Magician Chassis as the basic chassis for the robot ([](#fig:magician_chassis)).

We chose it because it has a double-decker configuration, and so we can put the battery in the lower part.

The chassis pack includes 2 DC motors and wheels as well as the structural part, in addition to a screwdriver and several necessary mechanical bits (standoffs, screws and nuts).

<div figure-id="fig:magician_chassis" figure-caption="The Magician Chassis">
     <img src="magician_chassis.jpg" style='width: 15em'/>
</div>

## Raspberry Pi 3 - Model B

Note: It is recommend to upgrade to Raspberry Pi 3 model B+. In this case the 5 GHz wireless adapter is no longer necessary.

The Raspberry Pi is the central computer of the Duckiebot. Duckiebots use Model B ([](#fig:rpi3b)) ( A 1.2GHz 64-bit quad-core ARMv8 CPU, 1GB RAM), a small but powerful computer.

<div figure-id="fig:rpi3b" figure-caption="The Raspberry Pi 3 Model B">
     <img src="rpi3b.png" style='width: 15em'/>
</div>

### Power Supply

We want a hard-wired power source (5VDC, 2.4A, Micro USB) to supply the Raspberry Pi ([](#fig:power_supply)) while not driving. This charger can double down as battery charger as well.

<div figure-id="fig:power_supply" figure-caption="The Power Supply">
     <img src="power_supply.png" style='width: 15em'/>
</div>

### Heat Sinks

The Raspberry Pi will heat up significantly during use. It is warmly recommended to add heat sinks, as in  [](#fig:heat_sinks). Since we will be stacking HATs on top of the Raspberry Pi with 15 mm standoffs, the maximum height of the heat sinks should be well below 15 mm. The chip dimensions are 15x15mm and 10x10mm.

<div figure-id="fig:heat_sinks" figure-caption="The Heat Sinks">
     <img src="heat-sinks.jpg" style='width: 15em'/>
</div>

###  Class 10 MicroSD Card

The MicroSD card ([](#fig:SDcard)) is the hard disk of the Raspberry Pi. 16 GB of capacity are sufficient for the system image.

<div figure-id="fig:SDcard" figure-caption="The MicroSD card">
     <img src="SDcard.png" style='width: 15em'/>
</div>

###  Mirco SD card reader

A microSD card reader ([](#fig:microsd-reader)) is useful to copy the system image to a Duckiebot from a computer to the Raspberry Pi microSD card, when the computer does not have a native SD card slot.

<div figure-id="fig:microsd-reader" figure-caption="The Mirco SD card reader">
     <img src="microsd-reader.png" style='width: 15em'/>
</div>

## Camera

The Camera is the main sensor of the Duckiebot. All versions equip a 5 Mega Pixels 1080p camera with wide field of view ($160^\circ$) fisheye lens ([](#fig:camera)).

<div figure-id="fig:camera" figure-caption="The Camera with Fisheye Lens">
     <img src="camera.png" style='width: 15em'/>
</div>

### Camera Mount

The camera mount ([](#fig:camera_mount)) serves to keep the camera looking forward at the right angle to the road (looking slightly down). The front cover is not essential.

<div figure-id="fig:camera_mount" figure-caption="The Camera Mount">
     <img src="camera-mount.jpg" style='width: 15em'/>
</div>

The assembled camera (without camera cable), is shown in ([](#fig:mounted-camera)).

<div figure-id="fig:mounted-camera" figure-caption="The Camera on its mount">
     <img src="mounted-camera.jpg" style='width: 15em'/>
</div>

### 300mm Camera Cable

A longer (300 mm) camera cable [](#fig:long_camera_cable) makes assembling the Duckiebot easier, allowing for more freedom in the relative positioning of camera and computational stack.

<div figure-id="fig:long_camera_cable" figure-caption="A 300 mm camera cable for the Raspberry Pi">
     <img src="long_camera_cable.png" style='width: 15em'/>
</div>


## Standoffs, Nuts and Screws

We use non electrically conductive standoffs (M2.5 12mm F 6mm M), nuts (M2.5), and screws (M2.5x10mm) to hold the Raspberry Pi to the chassis and the HATs stacked on top of the Raspberry Pi.

The Duckiebot requires 8 standoffs, 4 nuts and 4 screws.

<div figure-id="fig:stands_nuts_screws" figure-caption="Standoffs, Nuts and Screws">
     <img src="mech-bits.jpg" style='width: 15em'/>
</div>

## Zip Tie

Two 300x5mm zip ties are needed to keep the battery at the lower deck from moving around.

<div figure-id="fig:zipties" figure-caption="The zip ties">
     <img src="zipties.png" style='width: 15em'/>
</div>
