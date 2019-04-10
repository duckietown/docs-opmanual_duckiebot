# Indefinite Navigation {#demo-indefinite-navigation status=ready}

This is the description of the indefinite navigation demo.

Maintainer: Gianmarco Bernasconi

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

One (or possibly more) Duckiebot in [setup](#duckiebot-configurations) `DB-18`.

## Pre-flight checklist {#demo-indefinite-navigation-pre-flight}

Check: Sufficient battery charge of the Duckiebot.

Check: Duckiebot is properly calibrated.

## Demo instructions {#demo-indefinite-navigation-run}

Follow these steps to run the indefinite navigation demo on your Duckiebot:

Step 1: Power on your bot.

Step 2: Go to the portainer interface on

    http://![hostname].local:9000/#/containers

And check that only the necessary containers are running, namely:

    roscore
    dt18_00_basic_portainer_1
    dt18_01_health_stats_rpi-simple-server_1
    dt18_01_health_stats_rpi-health_1
    dt18_01_health_stats_rpi-duckiebot-loader_1
    dt18_01_health_stats_rpi-duckiebot-online_1
    dt18_00_basic_watchtower_1

If other containers are running, stop them.

Step 3: Run the base container:

    laptop $ docker -H ![hostname].local run -it --net host --privileged -v /data:/data --name base duckietown/rpi-duckiebot-base:megacity /bin/bash

A shell will open in the new container.

Step 4: Launch the demo in the container by:

    duckiebot-container $ source /docker/env.sh
    duckiebot-container $ roslaunch duckietown_demos indefinite_navigation.launch

Note: Many nodes need to be launched, so it will take quite some time.

Step 5: In a separate terminal, start a joystick with:

    laptop dts duckiebot keyboard_control ![hostname]

## Troubleshooting

Symptom: The Duckiebot fails at intersections.

Resolution: This problem is an open development problem, to improve the results tune the parameters of the `unicorn_intersection_node`.

Maintainer: Contact Gianmarco Bernasconi (ETHZ) via Slack for further assistance.
