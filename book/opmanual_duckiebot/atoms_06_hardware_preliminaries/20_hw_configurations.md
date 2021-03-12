# Understanding Duckiebot Configurations {#duckiebot-configurations status=ready}

<div class='requirements' markdown="1">

Requires: Nothing

Results: Knowledge of Duckiebot configuration naming conventions and their respective functionalities.

</div>

We define different Duckiebot configurations depending on their time of use and hardware components.

All Duckiebot configurations from `DB18` onward can be obtained [here](https://get.duckietown.com/).

## Duckiebot MOOC Founder's edition, or `DB21M` {#duckiebot-config-db21m}

The `DB21M` is the first Duckiebot equipped with a NVIDIA Jetson Nano 2 GB computational unit instead of a Raspberry Pi. The `DB21M` debuts in 2021 with the "[Self-Driving Cars with Duckietown](https://www.edx.org/course/self-driving-cars-with-duckietown)" massive open online course, hosted on the edX platform.

<div figure-id="fig:db21m" figure-caption="The Duckiebot version `DB21M`.">
   <img src="db21m.jpg" style='width: 20em'/>
</div>

The `DB21M` is readily recognized by its blazing blue chassis and triple-decker configuration. It is equipped with a sensor suite including: camera, time-of-flight sensor, inertial measurement unit (IMU) and wheel encoders. Moreover, the `DB21M` features new electronics (HUT v3.1, front and back bumpers), a screen, a button and a custom designed [Duckiebattery](#db-opmanual-dtbattery-v2) (not to be confused with the [Duckie-power-bank](#db-opmanual-dtbattery-v1)).

To assemble a `DB21M` Duckiebot, follow [these](#assembling-duckiebot-db21) instructions.

## Duckiebot version 2019, or `DB19` {#duckiebot-config-db19}

The `DB19` is the latest version of the Duckiebot. You have a `DB19` Duckiebot for sure if you have the blue motors shown in figure [](#fig:dc-motor-db19).

<div figure-id="fig:dc-motor-db19" figure-caption="The motors for the version `DB19`.">
   <img src="dc-motor-db19.png" style='width: 20em'/>
</div>

Apart from the new motors and another HUT (v. 2.1), the `DB19` is identical with the `DB18`. A complete version can be seen here:

<div figure-id="fig:db19-complete-cad" figure-caption="The complete Duckiebot 19">
   <img src="db19-complete-cad.png" style='width: 20em'/>
</div>

To assemble a `DB19` Duckiebot, follow [these](#assembling-duckiebot-db19) instructions.

## Duckiebot version 2018, or `DB18` {#duckiebot-config-db18}

You have a `DB18` Duckiebot if, e.g., you have pledged to the Kickstarter.

There are two configuration of the `DB18`.

### The `DB18` configuration

The main configuration is labeled plainly as `DB18` and is designed to operate on any Duckietown. You have the `DB18` if, e.g., you are a student attending the 2019 graduate level classes in ETH or the University of Montreal, or you have pledged to Summer 2018 Kickstarter.

The `DB18` supports different power bank models depending on the geographical region, but all these solutions are functionally equivalent, although their form factor is different.

You can recognize a `DB18` from previous versions for having only one board in addition to the Raspberry Pi, a backplate, and the computational stack mounted in the bottom deck.

<div figure-id="fig:db18-battery1" figure-caption="A Duckiebot DB18 assembly.">
   <img src="howto_assemble_finish_milestone.jpg" style='width: 20em'/>
</div>

<div figure-id="fig:db18-battery2" figure-caption="Another Duckiebot DB18 assembly, with a different battery.">
   <img src="howto_assemble_finish_milestone-2.jpg" style='width: 20em'/>
</div>

To assemble a `DB18` Duckiebot, follow [these](#assembling-duckiebot-db18) instructions.

### The `DB18-Robotarium` configuration

The `DB18-Robotarium` configuration adds to the `DB18` the hardware necessary to operate in Robotariums (a.k.a. Duckietown Autolabs): continuously operating Duckietowns. They are otherwise identical to the `DB18`.

The additional hardware consists of a top localization April Tag infrastructure and an "auto-charging" mod, which allows Duckiebots to dock to charging stations and estimate the residual battery charge.  

Robotariums are experimental Duckietown features, currently under development. You will find `DB18-Robotarium` models in university research labs.

If you are interested in obtaining `DB18-Robotarium` Duckiebots, or in building your Duckietown Robotarium, contact the [Duckietown team](mailto:info@duckietown.com).

<div figure-id="fig:db18-robotarium" figure-caption="A Duckiebot in DB18-Robotarium configuration.">
   <img src="a-glimpse-in-the-robotariums.png" style='width: 25em'/>
</div>

## Older Duckiebot versions {#duckiebot-config-db17}

For other Duckiebots, see [older versions](https://docs.duckietown.org/) of the book.
