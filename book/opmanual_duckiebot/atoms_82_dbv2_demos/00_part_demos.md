# Duckiebot V2 Demos {#part:dbv2-demos status=beta}

<div class='requirements' markdown='1'>

Requires: A Duckiebot fully setup and calibrated according to [](#dbv2)

Results: Learn how to run demos.

</div>

The following demos from Daffy for DB18 also work on DBV2:

 - Lane following with vehicle avoidance/following using camera
 - Indefinite navigation
 
In addition, the following new demos have been added, which use the new sensors on DBV2:

 - Lane following using line following sensors
 - Vehicle avoidance/following using ToF sensors
 - Closed loop speed control using the wheel encoder

## Launching demos {#dbv2-demos-launching}

All of the demos, including lane following and indefinite navigation, require new launch files. Therefore, the
standard procedure for running demos (As described in [](#running-demos)) will not immediately work for DBV2.
Until DTS is updated to cope with this, a different procedure is recommended for DBV2:

**Step 1**: Use Portainer to make sure `car-interface-dbv2` and `duckiebot-interface-dbv2` containers are running,
and that no containers based on `dt-core-dbv2` are running.

**Step 2**: Launch an interactive Docker container using `dt-core-dbv2`:

    $ docker -H ![duckiebot name].local run \
      --privileged -it -v /data:/data --name core-dbv2 \
      --network=host --rm 
      duckietown/dt-core-dbv2:daffy-arm32v7 bash
 
**Step 3**: Use `roslaunch` to launch the demo:

    $ roslaunch duckietown_demos_dbv2 ![demo name].launch ![arguments]

The possible values of `![demo name]` and `![arguments]` will be described in the following sections.