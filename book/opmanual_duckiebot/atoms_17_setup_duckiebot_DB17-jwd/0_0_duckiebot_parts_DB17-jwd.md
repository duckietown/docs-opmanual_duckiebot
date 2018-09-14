# Acquiring the parts (`DB17-jwd`) {#acquiring-parts-c0 status=ready}

The trip begins with acquiring the parts. Here, we provide a link to all bits and pieces that are needed to build a `DB17-jwd` Duckiebot, along with their price tag. If you are wondering what is the difference between different Duckiebot configurations, read [this](#duckiebot-configurations).

In general, keep in mind that:

- The links might expire, or the prices might vary.
- Shipping times and fees vary, and are not included in the prices shown below.
- International deliveries are subject to additional custom clearances and import fees.
- Substitutions are OK for the mechanical components,
  and not OK for all the electronics, unless you are OK in writing
  some software. Limited technical support will be offered for hardware customizations.
- Buying the parts for more than one Duckiebot makes each one cheaper than buying only one.
- For some components, the links we provide contain more bits than actually needed.


<div class='requirements' markdown="1">

Requires: Cost: USD 174 + Shipping Fees (minimal configuration `DB17`)

Requires: Time: 15 days (average shipping to the USA for cheapest choice of components)

Results: A kit of parts ready to be assembled in a `DB17` or `DB17-wjd` configuration.

Next steps: After receiving these components, you are ready to do some [soldering](#soldering-boards-c0) before [assembling](#assembling-duckiebot-db17-ttic) your `DB17` or `DB17-wjd` Duckiebot.

</div>

## Bill of materials

<!--
(<s>[4 Spacers (M3x5)](https://tinyurl.com/y9sjzm4r)</s><s>USD</s>)
(<s>[4 Screws (M3x10)](https://tinyurl.com/y9sjzm4r)</s><s>USD</s>)
(We can make the minimal configuration cheaper by USD20 removing the 16GB Class 10 MicroSD Card and stick with default)
-->

<div markdown="1">

 <col2 id='materials' figure-id="tab:materials" figure-caption="Bill of materials">
    <s>[Chassis](http://www.kr4.us/magician-chassis-rob-12866.html)</s>                         <s>USD 20</s>
    <s>[Camera with 160-FOV Fisheye Lens](https://tinyurl.com/ybwrcywc)</s>                         <s>USD 22</s>
    <s>[Camera Mount](https://tinyurl.com/ybyewdrt)</s>                         <s>USD 8.50</s>
    <s>[300mm Camera Cable](https://www.adafruit.com/product/1648)</s>                         <s>USD 2</s>
    <s>[Raspberry Pi 3 - Model B](https://tinyurl.com/ycsujzb9)</s>                         <s>USD 35</s>
    <s>[Heat Sinks](https://tinyurl.com/yanradnp)</s>                         <s>USD 2.90</s>
    <s>[Power supply for Raspberry Pi](https://www.adafruit.com/product/1995)</s>                         <s>USD 7.50</s>
    <s>[16 GB Class 10 MicroSD Card](https://tinyurl.com/ydawrgdx)</s>                         <s>USD 10</s>
    <s>[Mirco SD card reader](https://www.adafruit.com/product/939)</s><s>USD 6 </s>
    <s>[DC Motor HAT](https://tinyurl.com/y7qurpcw)</s>                         <s>USD 22.50</s>
    <s>[2 Stacking Headers](https://www.adafruit.com/product/2223)</s><s>USD 2.50/piece</s>
    <s>[Battery](https://tinyurl.com/ya7otc76)</s>                         <s>USD 20</s>
    <s>[16 Nylon Standoffs (M2.5 12mm F 6mm M)](https://tinyurl.com/y9uy73b2)</s>                         <s>USD 0.05/piece</s>
    <s>[4 Nylon Hex Nuts (M2.5)](https://tinyurl.com/ydy4znem)</s>                      <s>USD 0.02/piece</s>
    <s>[4 Nylon Screws (M2.5x10)](https://tinyurl.com/ya2uo9so)</s>                     <s>USD 0.05/piece</s>
    <s>[2 Zip Ties (300x5mm)](https://tinyurl.com/yb8v3nns)</s>                         <s>USD 9</s>
    <s>[Wireless Adapter (5 GHz)](https://tinyurl.com/ycvu7ok3) (`DB17-w`)</s><s>USD 20</s>
    <s>[Joypad](https://tinyurl.com/y9klooef) (`DB17-j`)</s>                         <s>USD 10.50</s>
    <s>[Tiny 32GB USB Flash Drive](https://tinyurl.com/ycao6men) (`DB17-d`)</s>                         <s>USD 12.50</s>
    <s>Total for `DB17` configuration</s>                         <s>USD 173.6</s>
    <s>Total for `DB17-w` configuration</s>                         <s>USD 193.6</s>
    <s>Total for `DB17-j` configuration</s>                         <s>USD 184.1</s>
    <s>Total for `DB17-d` configuration</s>                         <s>USD 186.1</s>
    <s>Total for `DB17-wjd` configuration</s>                         <s>USD 216.6</s>
 </col2>

</div>

<!--
<col2 figure-id="tab:materials-optionals" figure-caption="Bill of optional materials">
    <span>[Caster](https://tinyurl.com/y7gnesxn)</span>
    <span>USD 6.55/4 pieces</span>
    <span>[4 Standoffs (M3.5 12mm F-F)](https://tinyurl.com/ybd24t8c)</span>
    <span>USD 0.63/piece</span>
    <span>[8 Screws (M3.5x8mm)](https://tinyurl.com/ych4sfpa)</span>
    <span>USD 4.58/100 pieces</span>
    <span>[8 Split washer lock](https://tinyurl.com/y75onase)</span>
    <span>USD 1.59/100 pieces</span>
</col2>

<col2 id='materials-optionals' figure-id="tab:materials-optional" figure-caption="Bill of optional materials">
<s>[Caster](https://tinyurl.com/y7gnesxn)</s>                         <s>USD 6.55/4 pieces</s>
<s>[4 Standoffs (M3.5 12mm F-F)](https://tinyurl.com/ybd24t8c)</s>                         <s>USD 0.63/piece</s>
<s>[8 Screws (M3.5x8mm)](https://tinyurl.com/ych4sfpa)</s>                         <s>USD 4.58/100 pieces</s>
<s>[8 Split washer lock](https://tinyurl.com/y75onase)</s>                         <s>USD 1.59/100 pieces</s>
</col2>
-->

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

The Raspberry Pi is the central computer of the Duckiebot. Duckiebots use Model B ([](#fig:rpi3b)) ( A 1.2GHz 64-bit quad-core ARMv8 CPU, 1GB RAM), a small but powerful computer.

<div figure-id="fig:rpi3b" figure-caption="The Raspberry Pi 3 Model B">
     <img src="rpi3b.png" style='width: 15em'/>
</div>

### Power Supply

We want a hard-wired power source (5VDC, 2.4A, Micro USB) to supply the Raspberry Pi ([](#fig:power_supply)) while not driving. This charger can double down as battery charger as well.

<div figure-id="fig:power_supply" figure-caption="The Power Supply">
     <img src="power_supply.png" style='width: 15em'/>
</div>

Note: Students in the ETHZ-Fall 2017 course will receive a converter for US to CH plug.

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

## DC Motor HAT

We use the DC Stepper motor HAT ([](#fig:motor_hat)) to control the DC motors that drive the wheels. This item will require [soldering](#soldering-boards-c0) to be functional. This HAT has dedicate PWM and H-bridge for driving the motors.

<div figure-id="fig:motor_hat" figure-caption="The Stepper Motor HAT">
     <img src="motor_hat.png" style='width: 15em'/>
</div>

### Stacking Headers

We use a long 20x2 GPIO stacking header ([](#figure:stacking_header)) to connect the Raspberry Pi with the DC Motor HAT. This item will require [soldering](#soldering-boards-c0) to be functional.

<div figure-id="fig:stacking_header" figure-caption="The Stacking Headers">
     <img src="stacking_header.png" style='width: 15em'/>
</div>

## Battery

The battery ([](#fig:battery)) provides power to the Duckiebot.

We choose this battery because it has a good combination of size (to fit in the lower deck of the Magician Chassis), high output amperage (2.4A and 2.1A at 5V DC) over two USB outputs, a good capacity (10400 mAh) at an affordable price. The battery linked in the table above comes with two USB to microUSB cables.

<div figure-id="fig:battery" figure-caption="The Battery">
     <img src="battery-and-cables.jpg" style='width: 15em'/>
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

## Configuration `DB17-w`

### Wireless Adapter (5 GHz)

The Edimax AC1200 EW-7822ULC 5 GHz wireless adapter ([](#fig:edimax)) boosts the connectivity of the Duckiebot, especially useful in busy Duckietowns (e.g., classroom). This additional network allows easy streaming of images.  

<div figure-id="fig:edimax" figure-caption="The Edimax AC1200 EW-7822ULC wifi adapter">
     <img src="edimax.png" style='width: 15em'/>
</div>

## Configuration `DB17-j`

### Joypad

The joypad is used to manually remote control the Duckiebot. Any 2.4 GHz wireless controller (with a _tiny_ USB dongle) will do.

The model linked in the table ([](#fig:joystick)) does not include batteries.

<div figure-id="fig:joystick" figure-caption="A Wireless Joypad">
     <img src="joystick.png" style='width: 15em'/>
</div>

Requires: 2 AA 1.5V batteries ([](#fig:batteries)).

<div figure-id="fig:batteries" figure-caption="A Wireless Joypad">
     <img src="batteries.jpg" style='width: 15em'/>
</div>


## Configuration `DB17-d`

###  Tiny 32GB USB Flash Drive

In configuration `DB17-d`, the Duckiebot is equipped with an "external" hard drive ([](#fig:USBdrive)). This add-on is very convenient to store logs during experiments and later port them to a workstation for analysis. It provides storage capacity and faster data transfer than the MicroSD card.

<div figure-id="fig:USBdrive" figure-caption="The Tiny 32GB USB Flash Drive">
     <img src="USBdrive.png" style='width: 15em'/>
</div>
