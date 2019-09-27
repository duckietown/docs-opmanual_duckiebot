# Linux basics {#preliminaries-linux status=ready}

Linux is a group of free and open-source software operating systems built around the Linux kernel first released in 1991. Typically, Linux is packaged in a form known as a Linux distribution such as Fedora or Ubuntu (the latter one is what we use in this course).


## Ubuntu

As of this writing the most recent version of Ubuntu is 18.04 LTS (Long Term Service) which will be supported until April 2023.


## Installation

It is highly recommended to install Ubuntu directly on your laptop or as a dual boot operating system alongside your existing OS. However we also provide some guidance on installing Ubuntu within a Virtual Environment on your laptop.

### Dual Boot

* First you need to download a `.iso` image file which contains the version of Ubuntu you want. Here is [18.04 LTS](http://releases.ubuntu.com/18.04/) make sure to download the desktop image.
* Next, you need a free USB drive with at least 2GB of space. The drive will be completely written over.
* You need some software to write the .iso to the USB. If on Windows you can use [Rufus](https://rufus.ie/)
* Create the bootable USB drive, disconnect the USB then reconnect to your computer.
* Restart your computer
    - If your computer simply boots into the existing operating system you need to change the boot order in your BIOS.
    - Restart your computer again and press the button during startup which lets you into the BIOS. It may say on your computer what this button is but you may need to Google depending on your laptop model. For example Lenovo might be F1 or F2.
    - Look for an option to change boot order and put priority on your USB drive.
* Your computer should now boot into Ubuntu installation and you can follow the instructions for dual boot.

### Virtual Machine

* First you need to download a .iso image file which contains the version of Ubuntu you want. Here is [16.04 LTS](http://releases.ubuntu.com/16.04/) make sure to download the desktop image.
* Download you desired Virtual Machine such as Virtual Box

## Tutorial

[The Alternative](https://thealternative.ch/index.php?view=linuxdays) is a student club at ETHZ which has several tutorials on using Linux.

## Terminal

Some pointers:

* Open a terminal with Ctrl + Alt + T
* `/` is the top level root directoy which contains your
* `~` refers to your home folder located in `/home/![username]`
