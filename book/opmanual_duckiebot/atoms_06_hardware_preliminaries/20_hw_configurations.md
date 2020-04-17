# Understanding Duckiebot Configurations {#duckiebot-configurations status=ready}

<div class='requirements' markdown="1">

Requires: Nothing

Results: Knowledge of Duckiebot configuration naming conventions and their respective functionalities.

</div>

We define different Duckiebot configurations depending on their time of use and hardware components.

## Duckiebot version 2019, or `DB19` {#duckiebot-config-db19}

The `DB19` is the latest version of the Duckiebot. You have a `DB19` Duckiebot for sure if you have the blue motors shown in figure [](#fig:dc-motor-db19).

<div figure-id="fig:dc-motor-db19" figure-caption="The motors for the version `DB19`">
   <img src="dc-motor-db19.png" style='width: 25em'/>
</div>

Apart from the new motors and another HUT, the `DB19` is identical with the `DB18`. A complete version can be seen here:

<div figure-id="fig:db19-complete-cad" figure-caption="The complete Duckiebot 19">
   <img src="db19-complete-cad.png" style='width: 25em'/>
</div>


## Duckiebot version 2018, or `DB18` {#duckiebot-config-db18}

You have a `DB18` Duckiebot if, e.g., you have pledged to the Kickstarter.

There are two configuration of the `DB18`.

### The `DB18` configuration

The main configuration is labeled plainly as `DB18` and is designed to operate on any Duckietown. You have the `DB18` if, e.g., you are a student attending the 2019 graduate level classes in ETH or the University of Montreal, or you have pledged to Summer 2018 Kickstarter.

The `DB18` supports different power bank models depending on the geographical region, but all these solutions are functionally equivalent, although their form factor is different.

You can recognize a `DB18` from previous versions for having only one board in addition to the Raspberry Pi, a backplate, and the computational stack mounted in the bottom deck.

You can acquire the `DB18` hardware by contacting the [Duckietown Foundation](mailto:info@duckietown.org).

<div figure-id="fig:db18-battery1" figure-caption="A Duckiebot DB18 assembly.">
   <img src="howto_assemble_finish_milestone.jpg" style='width: 25em'/>
</div>

<div figure-id="fig:db18-battery2" figure-caption="Another Duckiebot DB18 assembly, with a different battery.">
   <img src="howto_assemble_finish_milestone-2.jpg" style='width: 25em'/>
</div>

### The `DB18-Robotarium` configuration

The `DB18-Robotarium` configuration adds to the `DB18` the hardware necessary to operate in Robotariums: continuously operating Duckietowns. They are otherwise identical to the `DB18`.

The additional hardware consists of a top localization April Tag infrastructure and an "auto-charging" mod, which allows Duckiebots to dock to charging stations and estimate the residual battery charge.  

Robotariums are experimental Duckietown features, currently under development. You will find `DB18-Robotarium` models in university research labs.

If you are interested in obtaining `DB18-Robotarium` Duckiebots, or in building your Duckietown Robotarium, contact the [Duckietown Foundation](mailto:info@duckietown.org).

<div figure-id="fig:db18-robotarium" figure-caption="A Duckiebot in DB18-Robotarium configuration.">
   <img src="a-glimpse-in-the-robotariums.png" style='width: 25em'/>
</div>

## Duckiebot versions 2017, or `DB17` {#duckiebot-config-db17}

In the  `DB17` version, we had several several different configurations.

The configurations are defined with a root: `DB17-`, indicating the "bare bones" Duckiebot used in the Fall 2017 synchronized course, and an appendix `y` which can be the union (in any order) of any or all of the elements of the optional hardware set $\aset{O} = \{$`w`, `j`, `d`, `p`, `l`, `c`$\}$.

A `DB17` Duckiebot can navigate autonomously in a Duckietown, but cannot communicate with other Duckiebots.

The elements of $\aset{O}$ are labels identifying optional hardware that aids in the development phase and enables the Duckiebot to talk to other Duckiebots. The labels stand for:

- `w`: 5 GHz **w**ireless adapter to facilitate streaming of images;

- `j`: wireless **j**oypad that facilitates manual remote control;

- `d`: USB **d**rive for additional storage space;

- `c`: a different **c**astor wheel to _replace_ the preexisting omni-directional wheel;

- `p`: **P**WM hat for convenient powering of the DC motor hat;

- `l`: includes **L**EDs, LED hat, bumpers and the necessary mechanical bits to set the bumpers in place. Note that the installation of the bumpers induces the _replacement_ of a few `DB17` components;

Note: During the Fall 2017 course, three Duckietown Engineering Co. branches (Zurich, Montreal, Chicago) are using these configuration naming conventions. Moreover, all institutions release hardware to their Engineers in training in two phases.

<!--
For information on acquiring the parts for these older configurations please see [`DB17-wjd`](https://docs.duckietown.org/DT17/opmanual_duckiebot/out/acquiring_parts_c0.html) or [`DB17-wjdlc`](https://docs.duckietown.org/DT17/opmanual_duckiebot/out/acquiring_parts_c1.html).
-->

It may be convenient at times to refer to hybrid configurations including any of the `DB17-jwcd` in conjunction with a _subset_ of the `DB17-l` components. In order to disambiguate, we define partial upgrades as:

- `DB17-l1`: _adds_ a PWM hat to `DB17`, in addition to a short USB angled power cable and a M-M power wire;
- `DB17-l2`: _adds_ a bumpers set to `DB17`, in addition to the mechanical bits to assemble it;
- `DB17-l3`: _adds_ a LED hat and 5 RGB LEDs to `DB17-l1l2`, in addition to the F-F wires to connect the LEDs to the LED board.

Note: introducing the PWM hat in `DB17-l1` induces a _replacement_ of the [spliced cable](#assembling-duckiebot-db17-cable-splitting) powering solution for the DC motor hat. Details can be found in [](#assembling-duckiebot-db17).


- **Functions**: `DB17-l` is the necessary configuration to enable communication between Duckiebots, hence fleet behaviors (e.g., negotiating the crossing of an intersection). Subset configurations are sometimes used in a standalone way for: (`DB17-l1`) avoid using a sliced power cable to power the DC motor hat in `DB17`, and (`DB17-l2`) for purely aesthetic reasons.
