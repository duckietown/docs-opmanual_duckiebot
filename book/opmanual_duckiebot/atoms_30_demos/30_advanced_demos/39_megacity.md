# Megacity {#demo-megacity status=beta}

This is the description of the great and marvelous megacity demo.

<div class='requirements' markdown="1">

Requires: Fully set up Duckiebots.

Requires: Fully set up Duckietown.

Results: One or more Duckiebot safely navigating in Duckietown.

</div>

Warning: This is an advanced demo. Make sure you have the experience and the hardware to fully capture its potential.

## Video of expected results {#demo-megacity-expected}

<div figure-id="fig:demo_succeeded-megacity">
    <figcaption>Outcome of a successful Megacity demo
    </figcaption>
    <dtvideo src='vimeo:329612860'/>
</div>

## Duckietown setup notes {#demo-megacity-duckietown-setup}

To run this demo, you can push your fantasy to the limit by building the Duckietown. The demo supports (almost) everything that is currently implemented. Make sure that your Duckietown complies with the appearance specifications presented in [the Duckietown specs](+opmanual_duckietown#dt-ops-appearance-specifications).


## Duckiebot setup notes {#demo-megacity-duckiebot-setup}

One (preferably more) Duckiebot in [setup](#duckiebot-configurations) `DB-18`.

## Pre-flight checklist {#demo-megacity-pre-flight}

Check: Sufficient battery charge of the Duckiebot.

Check: Duckiebot is properly calibrated.

## Demo instructions {#demo-megacity-run}

Note: You will have to repeat the instructions for each of the Duckiebots

Follow these steps to run the indefinite navigation demo on your Duckiebot:

**Step 1** Power on your bot and wait for the `duckiebot-interface` to initialize (the LEDs go off).

**Step 2**: Launch the demo by running:

    laptop $ dts duckiebot demo --demo_name megacity --duckiebot_name ![DUCKIEBOT_NAME] --package_name duckietown_demos

Note: Many nodes need to be launched, so it will take quite some time.

**Step 3**: With the joystick or In a separate terminal, start the joystick GUI:

    laptop $ dts duckiebot keyboard_control ![hostname]

and use the instructions to toggle between autonomous navigation and joystick control modes.

## Troubleshooting

Symptom: The Duckiebot fails at intersections.

Resolution: This problem is an open development problem, to improve the results tune the parameters of the `unicorn_intersection_node`, the procedure is explained in the [troubleshooting section](#trouble-unicorn_intersection).

Symptom: The demo fails, and it complains of a missing camera calibration file (usually is the `ground_projection_node` the first to die)

Resolution: Run the [camera calibration](#camera-calib) again.

Maintainer: Contact Gianmarco Bernasconi (ETHZ) via Slack for further assistance.
