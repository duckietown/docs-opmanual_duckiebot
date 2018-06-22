# Installing Ubuntu on laptops {#setup-laptop status=beta}

Assigned: Andrea Censi

Before you prepare the Duckiebot, you need to have a laptop with Ubuntu installed.

<div class='requirements' markdown='1'>

Requires: A laptop with free disk space.

Requires: Internet connection to download the Ubuntu image.

Requires: About 30 minutes.

Results: A laptop ready to be used for Duckietown.

</div>

## Install Ubuntu

Install Ubuntu 16.04.3.

See: For instructions, see for example [this online tutorial][tutorial].

[tutorial]: https://tutorials.ubuntu.com/tutorial/tutorial-install-ubuntu-desktop

**On the choice of username:**  During the installation, create a user for yourself with a username different from `ubuntu`, which is the default. Otherwise, you may get confused later.


<!--

I chose the following options:

        language: English
        username: ubuntu
        password: ubuntu
        hostname: duckietop

If you choose a different username, you will need to change all the commands later. -->

## Install useful software

Use `etckeeper` to keep track of the configuration in `/etc`:

    laptop $ sudo apt install etckeeper

Install `ssh` to login remotely and the server:

    laptop $ sudo apt install ssh

Use `byobu`:

    laptop $ sudo apt install byobu

Use `vim`:

    laptop $ sudo apt install vim

Use `htop` to monitor CPU usage:

    laptop $ sudo apt install htop

Additional utilities for `git`:

    laptop $ sudo apt install git git-extras

Other utilities:

    laptop $ sudo apt install avahi-utils ecryptfs-utils


## Install ROS

Install ROS on your laptop.

See: The procedure is given in [](#install-ROS).


## Other suggested software

### Redshift

This is Flux for Linux. It is an accessibility/lab safety issue: bright screens damage eyes and perturb sleep [](#bib:tosini16).

<cite id='bib:tosini16'>
    Tosini, G., Ferguson, I., Tsubota, K. <em>Effects of blue light on the circadian system and eye physiology</em>. Molecular Vision, 22, 61–72, 2016 (<a href="https://www.ncbi.nlm.nih.gov/pmc/articles/PMC4734149/">online</a>).
</cite>

Install redshift and run it.

    laptop $ sudo apt install redshift-gtk

Set to "autostart" from the icon (on the panel - near wifi/lan).

## Passwordless `sudo`

Set up passwordless `sudo`.

See: This procedure is described in [](#howto-passwordless-sudo).


Comment: Huh I don't know - this is great for usability, but horrible for security. If you step away from your laptop for a second and don't lock the screen, a nasty person could `sudo rm -rf /`. -FG

## SSH and Git setup


### Basic SSH config

Do the basic SSH config.

See: The procedure is documented in [](#ssh-local-configuration).


### Create key pair for `![username]`

Next, create a private/public key pair for the user; call it `![username]@![robot name]`.

See: The procedure is documented in [](#howto-create-key-pair).

### Add `![username]`'s public key to Github

Add the public key to your Github account.

See: The procedure is documented in [](#howto-add-pubkey-to-github).

If the step is done correctly, this command should succeed:

    duckiebot $ ssh -T git@github.com

### Local Git setup

Set up Git locally.

See: The procedure is described in [](#howto-git-local-config).

## Installation of the duckuments system

Optional but very encouraged: install the duckuments system.
This will allow you to have a local copy of the documentation
and easily submit questions and changes.

See: The procedure is documented in [](#duckumentation-installing-docs-system).


