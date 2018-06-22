# `DB17-l` setup {#leds-setup status=deprecated}


Note: We are currently revising the Duckiebot configurations for the upcoming semester, when Duckietown 1.0 will be officially released. Stay tuned!

TODO for Jacopo Tani: update configurations

<div class='requirements' markdown="1">

Requires: Duckiebot `DB17-lc` parts.
The acquisition process is explained in [](#acquiring-parts-c1).


Requires: Soldering `DB17-lc` parts.
The soldering process is explained in [](#soldering-boards-c1).

Requires: Having assembled PWM Hat on the Duckiebot with configuration `DB17-wjd`. The assembly process is explained in [](#assembling-duckiebot-c1).

Requires: Time: about 15 minutes.

Results: A Duckiebot with LEDs attached (configuration DB17-l3)

</div>


## Locate all required parts

To attach the LEDs on the duckiebots, the following components are needed:

* Soldered LSD board with Jumper
* LED lampes (5x)
* Female-female wires (20x)
* Nylon standoffs M2.5*12
* some Tape

<div figure-id="fig:LED_components" figure-caption="Components-List for LED configuration">
    <img src="LED_components.jpg" style='width:30em; height:auto'/>
</div>


## Connecting Wires to LEDs

### LEDs

The LEDs are common anode type. The longest pin is called the common.
The single pin on the side of common is red channel.
The two other pins are Green and Blue channels, with the blue furthest from the common pin.


<div figure-id="fig:LED_pins" figure-caption="LED pins with its functions">
    <img src="LED_Pins.png" style='width:20em; height:auto'/>
</div>

Use the long wires with two female ends. Attach one to each of the pins on the LED.

To figure out the order to connect them to the LSD hat, use the legend on the silkscreen and the information above. i.e. RX - means the red pin, CX - means the common, GX means the green, and BX means the blue. The "X" varies in number from 1-5 depending on which LED is being connected as discussed in the next section.

<div figure-id="fig:LSD_board" figure-caption="Locate 5 groups of 4 pins with label RX, C, GX, BX) on the LSD board">
    <img src="LSD_board.jpg" style='width:30em; height:auto'/>
</div>

Better: Use Tape to keep the LEDs stick with the wires.

## Connecting LEDs to LSD Hat

Silkscreen legend: Rx, Gx, Bx are red, green, and blue channels, accordingly, where x is the LED number; C is a common line (either common anode or common cathode).


Define the following names for the lights:

* “top” = top light  - the “top” light is now at the bottom
* fl  = front left
* fr  = front right
* br = back right
* bl = back left


The LEDs are wired according to [](#fig:LED_connections).

<div figure-id="fig:LED_connections">
    <img src="LED_connections.png" style='width:20em; height:auto'/>
</div>

Mappings from the numbers on the LED hats to the positions shown (TOP is now the one in the middle at the front)

* FR -> 5
* BR -> 4
* TOP -> 3
* BL -> 2
* FL -> 1

## Running the Wires Through the Chassis

It is advised that the LED cables are routed through the positions noted in the images below before installing the bumpers:

Front Left, Front Middle, and Front Right LED Wiring suggestion:

<div figure-id="fig:bumper_figure_0">
    <img src="image_0-1.jpg" style='width:20em; height:auto'/>
</div>


## Final tweaks

Adjust the LED terminals (particularly in the front) so that they do not interfere with the wheels. This can be accomplished by bending them up, away from the treads.
