# Setup - Laptop {#laptop-setup status=ready}

<!--
This page is for the Duckiebot `DB18` configuration.

For previous year's instructions, see [here](https://docs.duckietown.org/DT17/).

-->

<div class='requirements' markdown='1'>

Requires: A Ubuntu 18.04 or 20.04 environment with sufficient free disk space (recommended 100 GB).

Requires: Internet connection.

Requires: About 10 minutes.

Results: A laptop ready to be used for Duckietown.

</div>

## Minimal Laptop Requirements {#laptop-setup-reqs}

These installation steps make sure that you have a minimal "sane" environment, which includes:

1. Git and Git LFS;
2. Docker;
3. The Duckietown Shell.

## Native installation vs virtual machines {#laptop-setup-vms}

Having Ubuntu installed natively on your laptop is recommended but not strictly required.

If you are running Ubuntu in a VM make sure that you are using a Bridged network adapter (for example VirtualBox uses NAT by default). This allows you to be on the same subnetwork as your Duckiebot.

Sometimes when running a VMware machine in a Mac OS host, it is neccessary to have two network adapters: _Share with my Mac_ for connecting to the internet and _Bridged Networking_ for connecting to the Duckiebot.

For more information about networking with VMware, see [here](https://wiki.ros.org/ROS/NetworkSetup)

## Setup for Ubuntu 18.04 or 20.04 {#laptop-setup-ubuntu-18}

### System installation {#laptop-setup-ubuntu-18-system}

Install Ubuntu 18.04 or 20.04.

See: For instructions, see for example [this online tutorial][tutorial].

[tutorial]: https://tutorials.ubuntu.com/tutorial/tutorial-install-ubuntu-desktop

### Basic dependencies {#laptop-setup-ubuntu-18-basic}

Installs pip3, git, git-lfs, curl, wget:

    laptop $ sudo apt install -y python3-pip git git-lfs curl wget

Symptom: You see this output `E: Unable to locate package python3-pip`

Resolution: You will need to do `sudo apt update`

### Docker {#laptop-setup-ubuntu-18-docker}

Install Docker by following the instructions [here][docker_install].

[docker_install]: https://docs.docker.com/install/linux/docker-ce/ubuntu/

Adds user to "docker" group:

    laptop $ sudo adduser `whoami` docker

Note: you need to _log out and in_ for the group change take effect.

Warning: If you missed this step, you will later run into docker permission issues.

Make sure you have docker-compose installed:

    laptop $ sudo apt-get install docker-compose

### Duckietown Shell {#laptop-setup-ubuntu-18-shell}

Note: If you are not using Ubuntu environment and you would like to install Duckietown Shell, you should refer [HERE](https://github.com/duckietown/duckietown-shell)

Install Duckietown shell command by using the following command:

    laptop $ pip3 install --no-cache-dir --user -U duckietown-shell

After you have completed the above step, you will need to _log out and in_ for the shell command to take effect.

You can verify the command install by typing:

    laptop $ which dts

and it should output something like `/home/USER/.local/bin/dts`.

### Other optional configuration

Other useful packages:

    laptop $ sudo apt install emacs byobu openssh-server nfs-common zsh

Edit `~/.profile` and add:

    export EDITOR=emacs

### Z shell

    $ sudo apt install zsh
    $ chsh -s /usr/bin/zsh

Install `oh-my-zsh`:

    $ sh -c "$(wget https://raw.githubusercontent.com/robbyrussell/oh-my-zsh/master/tools/install.sh -O -)"

Edit `~/.zshrc` and set a different theme using:

    ZSH_THEME="bureau"

You can find other themes [at this page](https://github.com/robbyrussell/oh-my-zsh/wiki/themes).

Also add the line:

    . ~/.profile

#### Passwordless sudo

To add passwordless sudo:

    laptop $ sudo visudo

Change the line

    %sudo   ALL=(ALL:ALL) ALL

into

    %sudo   ALL=(ALL:ALL) NOPASSWD: ALL

### Other packages useful for development

    $ sudo apt install iotop atop htop

### For virtual machines

For VMWare, install this:

    laptop $ sudo apt install open-vm-tools

## Setting up Mac OS X {#laptop-setup-mac}

Warning: this configuration is not officially supported. There might be problems with implementation. We strongly urge you to use Ubuntu 20.04 enviornment.

### Basic dependencies {#laptop-setup-mac-basic}

You will need to find the instructions for git, git-lfs.

### Quartz {#laptop-setup-mac-quartz}

You will also need the latest version of XQuartz.

You can install using `brew` as follows:

    laptop $ brew cask install xquartz

Or, download from [here](https://www.xquartz.org/) and follow the instructions.

After installing XQuartz, run it in the command line with:

    laptop $ open -a XQuartz

Go to "Preferences" and in the security tab make sure that the checkbox next to "Allow" connections from network clients is set. Now close XQuartz.

You may want to add the following lines to your `.bashrc` file:

    export IP=$(ifconfig en0 | grep inet | awk '$1=="inet" {print $2}')
    xhost +$IP

Note: You may also need to add `/usr/X11/bin` to your system path in order for it to find `xhost`.

These will find your IP and then allow incoming connections to it in order to be able to popup windows from within docker containers.

Alternatively, you can run them each time before you want to use `X11` forwarding. This is done for you when you run things through the duckietown shell.

### Docker {#laptop-setup-mac-docker}

Follow [these instructions](https://docs.docker.com/docker-for-mac/install/)

### Duckietown Shell {#laptop-setup-mac-shell}

Follow [these instructions](https://github.com/duckietown/duckietown-shell)
 
