# Laptop Setup {#laptop-setup status=ready}

Assigned: Andrea Censi

This page is for the `DB18` configuration used in classes in 2018. 

For last year's instructions, see [here](https://docs.duckietown.org/DT17/).

<div class='requirements' markdown='1'>

Requires: A laptop with free disk space.

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

TODO: give some pointers for VM.
 
 
## Setup for Ubuntu 18 {#laptop-setup-ubuntu-18}

### System installation {#laptop-setup-ubuntu-18-system}

Install Ubuntu 18.

See: For instructions, see for example [this online tutorial][tutorial].

[tutorial]: https://tutorials.ubuntu.com/tutorial/tutorial-install-ubuntu-desktop

### Basic dependencies {#laptop-setup-ubuntu-18-basic}

Installs pip, git, git-lfs, docker, duckietown-shell:

    laptop $ sudo apt install -y python-pip git git-lfs curl wget 
    
### Docker {#laptop-setup-ubuntu-18-docker}

    laptop $ sudo apt install -y docker.io
    laptop $ sudo adduser `whoami` docker
    

Note: you need to *log in and out* to have the group change take effect.


### Duckietown Shell {#laptop-setup-ubuntu-18-shell}

To install the Duckietown shell, use:

    laptop $ pip install --user --no-cache-dir -U duckietown-shell

Edit the file `~/.profile` and add the line:

    export PATH=~/.local/bin:$PATH

Note: do not use `sudo pip` to install the Duckietown Shell.

Log out and in. This command should succeed:

    laptop $ dts version

### Other suggested configuration

Other useful packages:

    laptop $ sudo apt install vim byobu openssh-server nfs-common zsh

Edit `~/.profile` and add:

    export EDITOR=vim
    
### Z shell

    $ sudo apt install zsh
    $ chsh -s /usr/bin/zsh
    
Install `oh-my-zsh`:

    $ sh -c "$(wget https://raw.githubusercontent.com/robbyrussell/oh-my-zsh/master/tools/install.sh -O -)" 
 
Edit `~/.zshrc` and set a different theme using:

    ZSH_THEME="bureau"

You can find other themes [at this page](https://github.com/robbyrussell/oh-my-zsh/wiki/themes).

Also add the line:

    . ~/profile 


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

Assigned to: Liam

Note: this configuration is not officially supported.

### Basic dependencies {#laptop-setup-mac-basic}

You will need to find the instructions for git, git-lfs.

### Quartz {#laptop-setup-mac-quartz}

You will also need the latest version of XQuartz.

You can install using `brew` as follows:


    laptop $ brew cask install xquartz


Or, download from [here](https://www.xquartz.org/) and follow the instructions.


After installing XQuartz, run it in the command line with:


    laptop $ open -a XQuartz


Go to "Preferences" and in the security tab make sure that the checkbox next to "Allow" connections from network clients is set. 

It is also recommended that you add the following lines to your `.bashrc` file:


    export IP=$(ifconfig en0 | grep inet | awk '$1=="inet" {print $2}')
    xhost +$IP

These will find your IP and then allow incoming connections to it in order to be able to popup windows from within docker containers.

Alternatively, you can run them each time before you want to use `X11` forwarding.
 

### Docker {#laptop-setup-mac-docker}

Follow [these instructions](https://docs.docker.com/docker-for-mac/install/) 

### Duckietown Shell {#laptop-setup-mac-shell}

To install the Duckietown shell, use:

    laptop $ pip install --user --no-cache-dir -U duckietown-shell

Edit the file `~/.profile` and add the line:

    export PATH=~/.local/bin:$PATH

Note: do not use `sudo pip` to install the Duckietown Shell.

Log out and in. This command should succeed:

    laptop $ dts version



## Setup for Ubuntu 16.04 {#laptop-setup-ubuntu-16}

Note: It is suggested to use Ubuntu 18.

### System installation {#laptop-setup-ubuntu-16-system}

Install Ubuntu 16.04.3.

See: For instructions, see for example [this online tutorial][tutorial].

[tutorial]: https://tutorials.ubuntu.com/tutorial/tutorial-install-ubuntu-desktop

**On the choice of username:**  During the installation, create a user for yourself with a username different from `ubuntu`, which is the default. Otherwise, you may get confused later.


### Basic dependencies {#laptop-setup-ubuntu-16-deps}

Installs pip, git, git-lfs, docker, duckietown-shell:

    laptop $ sudo apt-get install software-properties-common  curl
    laptop $ sudo add-apt-repository ppa:git-core/ppa
    laptop $ curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
    laptop $ sudo apt-get install -y python-pip git git-lfs

### Docker {#laptop-setup-ubuntu-docker}

    laptop $ curl -fsSL https://get.docker.com | sudo bash
    laptop $ sudo usermod -aG docker `whoami`
    
Note: you need to *log in and out* to have the group change take effect.


### Duckietown Shell {#laptop-setup-ubuntu-16-shell}

Install the Duckietown Shell using:

    laptop $ pip install --no-cache-dir -U --user duckietown-shell

Edit the file `~/.profile` and add the line:

    export PATH=~/.local/bin:$PATH

Note: do not use `sudo pip` to install the Duckietown Shell.

 
Log out and in. This command should succeed:

    laptop $ dts version




