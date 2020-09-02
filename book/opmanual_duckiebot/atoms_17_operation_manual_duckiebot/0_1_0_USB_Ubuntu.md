# Getting a Ubuntu Environment {#USB-ubuntu status=ready}

This page instructs you how to get a Ubuntu Environment that is installed on a USB stick.

<div class='requirements' markdown='1'>

Requires: A USB Stick or SD Card with at least 32GB storage, recommended for 64GB.

Requires: A laptop with free disk space.

Requires: Internet connection.

Requires: Rufus installed on your current operating system.

Requires: About 20 minutes.

Results: A laptop that has Ubuntu 20.04 installed.

</div>

## VM vs. USB Stick vs. Creating a new partition vs. WSL

If you never have Ubuntu installed on your computer, typically you will have four options available to you.

You can create a new partition and dual boot your system. However, sometimes if you have a laptop with a small hardrive, it might be hard to create a designated partition just for Ubuntu. However, if you have the leisure of splitting your current partition in half, we strongly recommend you to create a dual boot machine. 

Note: If you want to install a dual boot machine, it is recommended for you to install Windows(Mac) first then Linux. If you do the other way, often time windows will not recognize the Ubuntu boot option and you have to later manually configure the boot options.

Warning: Always pay extra attention when you try to reconfigure your computer's partition as you don't want to lose any precious data.

You can also use virtual machine, to have duckietown environment installed in that. The problem is that with the added layer of virtual machine virtual networking, trouble shooting become a bit tedious. Virtual Machine also typically don't support hardware acceleration very well. 

Similarly to virtual machine approach, WSL, Windows Subsystem for Linux also has its challenge for both the hardware support (Not very well supporting Nvidia GPU acceleration as of writing this article in September 2020) as well as the network support. 

Using a USB Stick is the recommended method from us, as you do not need to have the risk of changing your pre-existing partition, nor worry about dual boot and Grub. Your computer hardware will get fully utilized just like when you install Ubuntu dual boot.

In this tutorial we will focus on installing Ubuntu on a USB stick

## Creating your Boot Stick

### Flashing Ubuntu onto your USB stick

In this tutorial we will be making a USB stick that contains a persistent partition. The persistent partition is the storage you will have available that does not erase itself on shutdown. As such making it as large as possible makes sense. **If you do not create a persistent partition, all work you do on your stick will be erased every time you power down.**

For details of creating your Ubuntu stick, follow the official Ubuntu tutorial:
https://ubuntu.com/tutorials/create-a-usb-stick-on-windows#1-overview

### Booting into your stick

Before you can boot into your stick you first need to take care of a few things:

- Disable *Fast startup* in your Windows settings (https://www.asus.com/us/support/FAQ/1031533/)
  - Fast startup makes it such that your computer doesn't fully power down (unless it updates), the BIOS however can only be accessed from a full power down.
- Figure out which keyboard key (or key combination) opens your BIOS. This is dependent on the manufacturer of your computer and is usually one of your *function* keys. Just google "[Computer Manufacturer] BIOS keys" and it should pop up. If you're having difficulty finding your key combination Windows provides the possibility to directly access the BIOS as well (https://store.hp.com/us/en/tech-takes/how-to-enter-bios-setup-windows-pcs, scroll down to *Method #2*)

NOTE: Typically, the BIOS menu key is F2,F12, or Del button, somtimes it can also be F1. For selecting Boot menu, it typically is F10 or F11.

NOTE: If you want to follow along, we suggest you open this tutorial on a mobile device or another computer. 

NOTE: If you have the option to boot using UEFI, you should use that for better compatibility as well as security.

Now let's get started!

- Turn off your computer

- Insert your boot USB

- As soon as you turn it back on immediately start rapidly and repeatedly pressing the previously determined BIOS key. Especially with new, quick computers the time window before boot starts is very small and you need to have the key pressed before then. This may take a few attempts before it works.

- If you pressed the correct key at the correct time the BIOS should open up for you. The way it looks again depends on the manufacturer of your computer, but it will usually be a very bare-bones GUI.

- Using your keyboard, navigate to the *boot priority* menu (It may be called differently in your BIOS so keep your eyes peeled for anything similar, or for anything with *Windows* in it).

- If your flashing procedure completed correctly, you should see both Windows and something with *USB*  or *Generic Storage Device* in it as your boot options. Move the USB above Windows. (Don't worry, if you later decide you don't want to work via the stick anymore this can be reversed at any time and often defaults back if no stick is recognized on boot.)

- Save/Apply and Exit your BIOS and let your computer boot

### First Boot

Before booting starts you may be prompted to choose between different actions that can be taken via your stick (Try Ubuntu, Install Ubuntu, etc.). Choose *Try Ubuntu* or simply *Ubuntu*.

- Before booting Ubuntu will preform a filesystem check. This may take a while, just be patient. (Alternatively you can stop it by pressing Ctrl+C, but this is strongly not recommended).

- On first boot Ubuntu will want to configure itself. Follow through the GUI and choose the language etc. you want your system to run.

- At the end of all of this you should have a fresh Ubuntu interface ready to go. 

