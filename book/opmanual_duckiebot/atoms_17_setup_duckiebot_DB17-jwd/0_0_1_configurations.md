# Duckiebot configurations {#duckiebot-configurations status=ready}

Note: We are currently revising the Duckiebot configurations for the upcoming semester, when Duckietown 1.0 will be officially released. Stay tuned!

TODO for Jacopo Tani: update configurations

<div class='requirements' markdown="1">

Requires: nothing

Results: Knowledge of Duckiebot configuration naming conventions and their respective functionalities.

</div>

We define different Duckiebot configurations depending on their time of use and hardware components. This is a good starting point if you are wondering what parts you should obtain to get started.

Once you have decided which configuration best suits your needs, you can proceed to the detailed descriptions for [`DB17-wjd`](#acquiring-parts-c0) or [`DB17-wjdlc`](#acquiring-parts-c1) Duckiebot.

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
