# Indefinite Navigation {#demo-indefinite-navigation status=ready}

This is the description of the indefinite navigation demo.

<div class='requirements' markdown="1">

Requires: [Wheel calibration](#wheel-calibration) completed.

Requires: [Camera calibration](#camera-calib) completed.

Requires: [Lane following](#demo-lane-following) demo has been successfully launched.

Requires: Fully set up Duckietown.

Results: One or more Duckiebot safely navigating in Duckietown.

</div>

## Video of expected results {#demo-indefinite-navigation-expected}

<!--
[link 1 of lane following](https://photos.google.com/share/AF1QipMEwYvBW5hl3_l4M0f9on3RSKJmYftbWxo0nSyW7EMTBWs7iXRc_fHEc5mouSMSxA/photo/AF1QipPOmXr0yu__d_J0Wefp1Gm6sNTtptUk57FvS6Fo?key=M1ZWc2k0Nnl4ckFjd3dwRmV0WmdMSzFWU0xmOXh3)
-->

<div figure-id="fig:demo-indefinite-navigation-video">
    <figcaption>Demo: indefinite navigation
    </figcaption>
    <dtvideo src='vimeo:247596730'/>
</div>

TODO: add a different video with an up to specification Duckietown.

## Duckietown setup notes {#demo-indefinite-navigation-duckietown-setup}

To run this demo, you can setup a quite complex Duckietown. The demo supports normal road tiles, intersections and traffic lights. Make sure that your Duckietown complies with the appereance specifications presented in [the Duckietown specs](+opmanual_duckietown#dt-ops-appearance-specifications). In particular correct street signaling is key to success of intersections handling.


## Duckiebot setup notes {#demo-indefinite-navigation-duckiebot-setup}

One (or possibly more) Duckiebot in configuration [DB18](#duckiebot-configurations).

## Pre-flight checklist {#demo-indefinite-navigation-pre-flight}

Check: Sufficient battery charge of the Duckiebot.

Check: Duckiebot is properly calibrated.

## Demo instructions {#demo-indefinite-navigation-run}

Follow these steps to run the indefinite navigation demo on your Duckiebot:

**Step 1**: Power on your bot and wait for the `duckiebot-interface` to initialize (the LEDs go off).

**Step 2**: Launch the demo by running:

    laptop $ dts duckiebot demo --demo_name indefinite_navigation --duckiebot_name ![DUCKIEBOT_NAME] --package_name duckietown_demos

Note: Many nodes need to be launched, so it will take quite some time.

**Step 3**: With the joystick or In a separate terminal, start the joystick GUI:

    laptop $ dts duckiebot keyboard_control ![hostname]

and use the instructions to toggle between autonomous navigation and joystick control modes.

## Troubleshooting

Symptom: The Duckiebot fails at intersections.

Resolution: This problem is an open development problem, to improve the results tune the parameters of the `unicorn_intersection_node`, the procedure is explained in the [troubleshooting section](#trouble-unicorn_intersection).

Maintainer: Contact Gianmarco Bernasconi (ETHZ) via Slack for further assistance.
