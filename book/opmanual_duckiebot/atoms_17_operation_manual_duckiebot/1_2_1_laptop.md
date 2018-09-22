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

## Minimal Laptop Requirements

These installation steps make sure that you have a minimal "sane" environment, which includes:

1. Git and Git LFS;
2. Docker;
3. The Duckietown Shell.

### Installation on Ubuntu 18.xx

Installs pip, git, git-lfs, docker, duckietown-shell:

    $ sudo apt install -y python-pip git git-lfs
    
    $ sudo apt install -y docker.io
    $ sudo adduser `whoami` docker
    
    $ sudo pip install --no-cache-dir -U duckietown-shell

Note: you need to *log in and out* to have the group change take effect.

### Installation on Ubuntu 16.xx

Installs pip, git, git-lfs, docker, duckietown-shell:

    $ sudo apt-get install software-properties-common  curl
    $ sudo add-apt-repository ppa:git-core/ppa
    $ curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
    $ sudo apt-get install  -y python-pip git git-lfs
    
    $ curl -fsSL https://get.docker.com | sudo bash
    $ sudo usermod -aG docker `whoami`
    
    $ sudo pip install --no-cache-dir -U duckietown-shell

Note: you need to *log in and out* to have the group change take effect.


### Installation in other operating systems

You will need to find the instructions for git, git-lfs, docker.

To install the shell, use:

    $ sudo pip install --no-cache-dir -U duckietown-shell

The shell itself does not require any other dependency beside standard cross-platform Python libraries.
