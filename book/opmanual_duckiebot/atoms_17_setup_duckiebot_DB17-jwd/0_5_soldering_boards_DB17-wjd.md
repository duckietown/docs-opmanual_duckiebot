# Soldering boards (`DB17`) {#soldering-boards-c0 status=deprecated}

Assigned: Shiying Li

<!--
Shiying: how long does it take to solder the stuff described in this chapter? (I guessed 30 mins, change it with the real number)

-->

<div class='requirements' markdown="1">

Requires: Parts: Duckiebot `DB17` parts. The acquisition process is explained in [](#acquiring-parts-c0). The configurations are described in [](#duckiebot-configurations). In particular:

- [GPIO Stacking Header](http://adafru.it/2223)
- [DC and Stepper Motor HAT for Raspberry Pi](http://adafru.it/2348)

Requires: Tools: Solderer

Requires: Experience: some novice-level experience with soldering.

Requires: Time: 30 minutes

Results: Soldered DC Motor HAT

</div>

Note: It is better to be safe than sorry. Soldering is a potentially hazardous activity. There is a fire hazard as well as the risk of inhaling toxic fumes. Stop a second and make sure you are addressing the safety standards for soldering when following these instructions. If you have never soldered before, seek advice.

## General tips

Note: There is a general rule in soldering: solder the components according to their height, from lowest to highest.

In this instruction set we will assume you have soldered something before and are acquainted with the soldering fundamentals. If not, before proceeding, read this great tutorial on soldering:

See: [Alternative instructions: how to solder on Headers and Terminal Block](https://learn.adafruit.com/adafruit-dc-and-stepper-motor-hat-for-raspberry-pi/assembly)

### Preparing the components

Take the GPIO stacking header [](#fig:GPIO_Stacking_Header) out of Duckiebox and sort the following components from DC motor HAT package:

- Adafruit DC/Stepper Motor HAT for Raspberry Pi

- 2-pin terminal block (2x), 3-pin terminal block (1x)

<div figure-id="fig:GPIO_Stacking_Header" figure-caption="GPIO_Stacking_Header">
     <img src="GPIO_Stacking_Header.jpg" style='width: 30ex; height: auto'/>
</div>

<div figure-id="fig:DC/Stepper_HAT" figure-caption="DC/Stepper Motor HAT and solder components">
    <img src="DC_stepper_HAT.jpg" style='width: 30ex; height: auto'/>
</div>

### Soldering instructions

1) Make a 5 pin terminal block by sliding the included 2 pin and 3 pin terminal blocks into each other [](#fig:terminal_block).

<div figure-id="fig:terminal_block" figure-caption="5 pin terminal_block">
   <img src="terminal_block.jpg" style='width: 30ex; height: auto'/>
</div>

2) Slide this 5 pin block through the holes just under "M1 GND M2" on the board. Solder it on (we only use two motors and do not need connect anything at the "M3 GND M4" location) ([](#figure:upview_Stepper_Motor));

3) Slide a 2 pin terminal block into the corner for power. Solder it on. ([](#figure:sideview_terminal));

4) Slide in the GPIO Stacking Header onto the 2x20 grid of holes on the edge opposite the terminal blocks and with vice versa direction ([](#figure:GPIO_HAT_orientation)). Solder it on.

Note: stick the GPIO Stacking Header from bottom to top, different orientation than terminal blocks (from top to bottom).


<div figure-id="fig:GPIO_HAT_orientation" figure-caption=" ">
   <img src="GPIO_HAT_orientation.jpg" style='width: 30ex; height: auto'/>
</div>

<div figure-id="fig:sideview_terminal" figure-caption="Side view of finished soldering DC/Stepper Motor HAT">
   <img src="sideview_Stepper_HAT.jpg" style='width: 30ex; height: auto'/>
</div>

<div figure-id="fig:upview_Stepper_Motor" figure-caption="upside view of finished soldering DC/Stepper Motor HAT">
   <img src="upview_stepper_Motor.jpg" style='width: 30ex; height: auto'/>
</div>
