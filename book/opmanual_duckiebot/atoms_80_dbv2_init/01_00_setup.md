# Setup {#dbv2-setup status=beta}

<div class='requirements' markdown='1'>

Requires: A Duckiebot in configuration `DBV2`

Requires: A laptop with free disk space.

Requires: Internet connection.

Results: A DBV2 ready for calibration.

</div>

The setup for the `DBV2` software is very similar to that of `DB18`, as described in [](#building-duckiebot-c0)

The main differences are in which containers you should run, and what calibration routines you must do.

## Stop Old Containers

After initializing your `DBV2`, open Portainer, and remove the following containers, if they exist:

 - `duckiebot-interface`
 - `car-interface`
 - `core`

The `DBV2` relies on special versions of each of these containers. These new containers are built on top of the
original containers, and therefore reuse most of the same code. This is why these old containers must be deleted.

## Get new images

The next step is to get these new docker images on your `DBV2`. There are two ways to do this:

#### Option 1: From Docker Hub {status=beta}

TODO: Upload the images to Docker Hub. If you are reading this, I have not done that yet, and so this option is
not yet available.

Run the following commands on your computer:

    $ docker -H ![duckiebot name].local pull duckietown/dt-duckiebot-interface-dbv2:daffy-arm32v7
    $ docker -H ![duckiebot name].local pull duckietown/dt-car-interface-dbv2:daffy-arm32v7
    $ docker -H ![duckiebot name].local pull duckietown/dt-core-dbv2:daffy-arm32v7
    
#### Option 2: Build from source

First, clone the repositories 
[dt-duckiebot-interface-dbv2](https://github.com/duckietown/dt-duckiebot-interface-dbv2),
[dt-car-interface-dbv2](https://github.com/duckietown/dt-car-interface-dbv2), and
[dt-core-dbv2](https://github.com/duckietown/dt-core-dbv2). Then, in each repository folder, run the following command:

    $ dts devel build -H ![duckiebot name].local
    

## Create Docker containers

Similarly to `DB18`, `dt-duckiebot-interface-dbv2` and `dt-car-interface-dbv2` should almost always be running.
Use the following commands to create one container for each image, which will automatically start every time
your `DBV2` boots:

    $ docker -H ![duckiebot name].local run \
      --privileged -dit -v /data:/data --name duckiebot-interface-dbv2 --network=host \
      --restart unless-stopped duckietown/dt-duckiebot-interface-dbv2:daffy-arm32v7
    
    $ docker -H ![duckiebot name].local run \
      --privileged -dit -v /data:/data --name car-interface-dbv2 --network=host \
      --restart unless-stopped duckietown/dt-car-interface-dbv2:daffy-arm32v7

The `dt-core-dbv2` container will be started later, for running demos. See: [](#dbv2-demos)

## Verify Initialization

To verify that everything is setup properly, try using keyboard control, as described in [](#rc-control).
