# Setup - Laptop {#laptop-setup status=ready}

<!--
This page is for the Duckiebot `DB18` configuration.

For previous year's instructions, see [here](https://docs.duckietown.org/DT17/).

-->

<div class='requirements' markdown='1'>

Requires: An Ubuntu 20.04 environment with sufficient free disk space (recommended 100 GB).

Requires: Internet connection.

Requires: About 10 minutes.

Results: A laptop ready to be used for Duckietown.

</div>

## Minimal Laptop Requirements {#laptop-setup-reqs}

These installation steps make sure that you have a minimal "sane" environment, which includes:

1. Git;
2. Docker;
3. The Duckietown Shell.

## Native installation vs virtual machines {#laptop-setup-vms}

Having Ubuntu installed natively on your laptop is recommended but not strictly required.

If you are running Ubuntu in a VM make sure that you are using a Bridged network adapter (for example VirtualBox uses NAT by default). This allows you to be on the same subnetwork as your Duckiebot.

Sometimes when running a VMware machine on a Mac OS host, it is neccessary to have two network adapters: _Share with my Mac_ for connecting to the internet and _Bridged Networking_ for connecting to the Duckiebot.

For more information about networking with VMware, see [here](https://wiki.ros.org/ROS/NetworkSetup).

## Setup for Ubuntu 20.04 {#laptop-setup-ubuntu}

### System installation {#laptop-setup-ubuntu-system}

Install Ubuntu 20.04.

Note: *if you have a robot, you must install Ubuntu natively*.

See: For instructions, see for example [this online tutorial][tutorial].

[tutorial]: https://tutorials.ubuntu.com/tutorial/tutorial-install-ubuntu-desktop

### Basic dependencies {#laptop-setup-ubuntu-basic}

Installs pip3, git, git-lfs, curl, wget:

    laptop $ sudo apt update
    laptop $ sudo apt install -y python3-pip git git-lfs curl wget


### Docker {#laptop-setup-ubuntu-docker}

Install Docker by following the instructions [here][docker_install].

Note: If you already have docker installed, it is strongly recommended that you upgrade your version of docker to the latest one by reinstalling based on these instructions. If you later see an error that you don't have the sufficient version of _buildx_ this is the root cause.   

[docker_install]: https://docs.docker.com/install/linux/docker-ce/ubuntu/

Adds user to "docker" group:

    laptop $ sudo adduser `whoami` docker

Note: you need to _log out and back in_ for the group change to take effect.

Make sure you have docker-compose installed:

    laptop $ sudo apt-get install docker-compose
    
Make sure the Docker version is `v.0.8.0+` and `buildx` version `v1.4.0+` through:

    docker --version

    docker buildx --version
    
Warning: If you missed this step, you will later run into docker permission issues.

### Duckietown Shell {#laptop-setup-ubuntu-shell}

Note: If you are not using Ubuntu environment and you would like to install Duckietown Shell, you should refer to [this page](https://github.com/duckietown/duckietown-shell).

Install the Duckietown Shell using the following command,

    laptop $ pip3 install --no-cache-dir --user --upgrade duckietown-shell

The first thing you need to do with the Duckietown Shell, is set the Duckietown software 
distribution you want to work with, for this version of the book, we use `daffy`. 
Set the shell to use the `daffy` distribution by running the following command, 

    laptop $ dts --set-version daffy

### For virtual machines

For VMWare, install the package `open-vm-tools`:

    laptop $ sudo apt install open-vm-tools

## Setting up Mac OS X {#laptop-setup-mac}

Warning: this configuration is not officially supported. There might be problems with implementation. We strongly urge you to use Ubuntu 20.04 enviornment.

### Basic dependencies {#laptop-setup-mac-basic}

You will need to find the instructions for git, git-lfs.

### Quartz {#laptop-setup-mac-quartz}

You will also need the latest version of XQuartz.

You can install using `brew` as follows:

    laptop $ brew install xquartz

Or, download from [here](https://www.xquartz.org/) and follow the instructions.

After installing XQuartz, run it in the command line with:

    laptop $ open -a XQuartz

Go to "Preferences" and in the "Security" tab make sure that the checkbox next to "Allow connections from network clients" is set. Now close XQuartz.

You may want to add the following lines to your `.bashrc` file:

    export IP=$(ifconfig en0 | grep inet | awk '$1=="inet" {print $2}')
    xhost +$IP

Note: You may also need to add `/usr/X11/bin` to your system path in order for it to find `xhost`.

These will find your IP and then allow incoming connections to it in order to be able to popup windows from within docker containers.

Alternatively, you can run them each time before you want to use `X11` forwarding. This is done for you when you run things through the duckietown shell.

### Docker {#laptop-setup-mac-docker}

- (For Linux): Follow [these instructions](https://docs.docker.com/engine/install/ubuntu/);
- (For Mac): Follow [these instructions](https://docs.docker.com/docker-for-mac/install/);
- (Important): Follow [post-install instructions](https://docs.docker.com/engine/install/linux-postinstall/);

Note: If you are using Docker Desktop for Mac, you will need to specify the amount of memory for Docker to use in order to run the Duckietown containers. Open the Docker menu and go to "Preferences" then "Advanced". Use the slider in the Advanced tab to increase Memory to a minimum of 4.5GB.

### Duckietown Shell {#laptop-setup-mac-shell}

Follow [these instructions](https://github.com/duckietown/duckietown-shell).
 
