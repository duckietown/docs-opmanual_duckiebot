# Assembling the Duckiebot (`DB17-lc`) {#assembling-duckiebot-c1 status=deprecated}

Assigned: Jacopo Tani

<div class='requirements' markdown="1">

Requires: Duckiebot `DB17-lc` parts. The acquisition process is explained in [](#acquiring-parts-c1).

Requires: Soldering `DB17-lc` parts. The soldering process is explained in [](#soldering-boards-c1).

Requires: Having assembled the Duckiebot in configuration `DB17` (or any `DB17-wjd`). The assembly process is explained in [](#assembling-duckiebot-db17-ttic).

Requires: Time: about 30 minutes.

Results: An assembled Duckiebot in configuration `DB17-wjdlc`.

</div>

## Assembly the Servo/PWM hat (`DB17-l1`)

Recommend: If you have Bumpers or caster to assembly, it is recommend to have Bumpers and caster assembled before the PWM hat. The assembly process is explained in [](#assembling-duckiebot-db17-ttic).

### Locate the components for Servo/PWM hat

* Soldered PWM hat (1x)
* Nylon Standoffs (M3.5 12mm F-F) (4x)
* Power cable: short angled male USB-A to 5.5/2.1mm DC power jack cable
* Male-Male Jumper Wires (1x)
* Screwdriver

<div figure-id="fig:component_PWM" figure-caption="Components-List for PWM HAT">
     <img src="component_PWM.jpg" style='width: 30em'/>
</div>

### Remove the hand-made USB power cable from DC Motor HAT

From now on, the DC Motor Hat will be powered by the PWM HAT via male -male jumper wire. Before that, the previous hand-made USB power cable needed to be removed. Insert the male-male jumper wire into `+` power terminal on the DC motor HAT (DC-end).

<div figure-id="fig:positive_terminal_DC" figure-caption="Insert the male-male wire into `+` terminal block on the DC motor HAT">
     <img src="plus_terminal_DC.jpg" style='width: 30em'/>
</div>

### Stack the PWM HAT above the DC motor HAT

Put a soldered Servo/PWM HAT board (in your Duckiebox) with 4 standoffs on the top of Stepper Motor HAT.

Insert the other end of male-male jumper wire into "**+5**"V power terminal on the PWM HAT (PWM-end). It leads the power to DC motor HAT.

<div figure-id="fig:pos_terminal_PWM" figure-caption="Insert the PWM-end into +5V terminal on PWM HAT ">
     <img src="pos_terminal_PWM.jpg" style='width: 30em'/>
</div>


### Power Supply for PWM HAT

To power the PWM/Servo HAT from the battery, plugin a short (30cm) angled male USB-A to 5.5/2.1mm DC power jack cable into PWM HAT. The other end of the power cable will plugin to the battery when it is in use.  

<div figure-id="fig:angled_power_cable" figure-caption="Plugin the short angled male DC power cable">
     <img src="angled_power_cable.jpg" style='width: 30em'/>
</div>


## Assembling the Bumper Set (`DB17-l2`)

For instructions on how to assemble your bumpers set, refer to: [](#bumper-assembly).

## Assembling the LED HAT and LEDs (`DB17-l3`)

For instructions on how to assemble the LED HAT and related LEDs, refer to: [](#leds-setup).

TODO: finish above, estimate assembly time, add bumper assembly instructions, add LED positioning and wiring instructions, add castor wheel assembly instructions
