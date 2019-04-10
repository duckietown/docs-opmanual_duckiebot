# Megacity {#demo-megacity status=ready}

This is the description of the great and marvelous megacity demo.

Maintainer: Gianmarco Bernasconi

<div class='requirements' markdown="1">

Requires: Fully set up Duckiebots

Requires: Fully set up Duckietown.

Results: One or more Duckiebot safely navigating in Duckietown.

</div>

Warning: This demo is designed to be used in robotariums, by expert users. Make sure you have the experience and the hardware to fully capture the potential of the demo.

## Video of expected results {#demo-megacity-expected}

<div figure-id="fig:demo_succeeded-megacity">
    <figcaption>Outcome of a successful Megacity demo
    </figcaption>
    <dtvideo src='vimeo:329612860'/>
</div>

## Duckietown setup notes {#demo-megacity-duckietown-setup}

To run this demo, you can push your fantasy to the limit by building the Duckietown. The demo supports (almost) everything that is currently implemented. Make sure that your Duckietown complies with the appereance specifications presented in [the Duckietown specs](+opmanual_duckietown#dt-ops-appearance-specifications).


## Duckiebot setup notes {#demo-megacity-duckiebot-setup}

One (preferably more) Duckiebot in [setup](#duckiebot-configurations) `DB-18`.

## Pre-flight checklist {#demo-megacity-pre-flight}

Check: Sufficient battery charge of the Duckiebot.

Check: Duckiebot is properly calibrated.

## Demo instructions {#demo-megacity-run}

Note: You will have to repeat the instructions for each of the Duckiebots

Follow these steps to run the indefinite navigation demo on your Duckiebot:

**Step 1** Power on your bot.

**Step 2** Go to the portainer interface on:

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

**Step 3** Run the base container:

    laptop $ docker -H ![hostname].local run -it --net host --privileged -v /data:/data --name base duckietown/rpi-duckiebot-base:megacity /bin/bash

A shell will open in the new container.

**Step 4** Launch the demo in the container by:

    duckiebot-container $ source /home/software/docker/env.sh
    duckiebot-container $ roslaunch duckietown_demos megacity.launch

Note: Many nodes need to be launched, so it will take quite some time. Moreover the CPU load will be extremely high during the demo.

**Step 5** In a separate terminal, start a joystick with:

    laptop $ dts duckiebot keyboard_control ![hostname]


## Troubleshooting

Symptom: The Duckiebot fails at intersections.

Resolution: This problem is an open development problem, to improve the results tune the parameters of the `unicorn_intersection_node`, the procedure is explained in the [troubleshooting section](#trouble-unicorn_intersection).

Symptom: The demo fails, and it complains of a missing camera calibration file (usually is the `ground_projection_node` the first to die)

Resolution: Run the [camera calibration](#camera-calib) again.

Maintainer: Contact Gianmarco Bernasconi (ETHZ) via Slack for further assistance.
