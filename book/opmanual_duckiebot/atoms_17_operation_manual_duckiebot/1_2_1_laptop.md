# Laptop Setup {#laptop-setup status=draft}

Assigned: Andrea Censi

This page is for the `DB18` configuration used in classes in 2018. For last year's instructions see [here](docs.duckietown.org/17/).

<div class='requirements' markdown='1'>

Requires: A laptop with free disk space.

Requires: Internet connection.

Requires: About 10 minutes.

Results: A laptop ready to be used for Duckietown.

</div>

Having Ubuntu installed natively on your laptop is recommended but not strictly required.  



## Installing Ubuntu 16.04

Install Ubuntu 16.04.3.

See: For instructions, see for example [this online tutorial][tutorial].

[tutorial]: https://tutorials.ubuntu.com/tutorial/tutorial-install-ubuntu-desktop

**On the choice of username:**  During the installation, create a user for yourself with a username different from `ubuntu`, which is the default. Otherwise, you may get confused later.

<!--

I chose the following options:

```
    language: English
    username: ubuntu
    password: ubuntu
    hostname: duckietop
```

If you choose a different username, you will need to change all the commands later. -->

TODO: Breandan, is 18.04 supported?

#### Dual Booting

TODO:





## Minimal Laptop Requirements

These installation steps make sure that you have a minimal "sane" environment, which includes:

1. Git and Git LFS;
2. Docker;
3. The Duckietown Shell.

### Installation on Ubuntu 18.xx

Installs pip, git, git-lfs, docker, duckietown-shell:

    laptop $ sudo apt install -y python-pip git git-lfs
    laptop $ sudo apt install -y docker.io
    laptop $ sudo adduser `whoami` docker
    laptop $ sudo pip install --no-cache-dir -U duckietown-shell

Note: you need to *log in and out* to have the group change take effect.

### Installation on Ubuntu 16.xx

Installs pip, git, git-lfs, docker, duckietown-shell:

    laptop $ sudo apt-get install software-properties-common  curl
    laptop $ sudo add-apt-repository ppa:git-core/ppa
    laptop $ curl -s https://packagecloud.io/install/repositories/github/git-laptop lfs/script.deb.sh | sudo bash
    laptop $ sudo apt-get install  -y python-pip git git-lfs
    laptop $ curl -fsSL https://get.docker.com | sudo bash
    laptop $ sudo usermod -aG docker `whoami`
    laptop $ sudo pip install --no-cache-dir -U duckietown-shell

Note: you need to *log in and out* to have the group change take effect.

### Installation on Mac OSX 

TODO: Liam

You will need to find the instructions for git, git-lfs, docker.

To install the shell, use:

```
laptop $ sudo pip install --no-cache-dir -U duckietown-shell
```

You will also need the latest version of XQuartz:

```
laptop $ brew cask install xquartz
```

or download from [here](https://www.xquartz.org/) and follow the instructions.

after installing XQuartz, run it in the command line with:

```
laptop $ open -a XQuartz
```

Go to "Preferences" and in the security tab make sure that the checkbox next to "Allow" connections from network clients is set. 

It is also recommended that you add the following lines to your `.bashrc` file (or run them before you want to use `X11` forwarding):

`export IP=`&#36;`(ifconfig en0 | grep inet | awk '&#36;1=="inet" {print &#36;2}')`

`xhost + `&#36;`IP`

these will find your IP and hten allow incoming connections to it in order to be able to pop up windows from within docker containers. 

### Installation in other operating systems

At your own risk.


