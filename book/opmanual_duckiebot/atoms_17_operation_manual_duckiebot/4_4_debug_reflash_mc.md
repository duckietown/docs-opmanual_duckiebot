# Debug - Reflash Microcontroller {#reflash-microcontroller status=ready}

This page is only for the DB18 and DB19 configuration (not the DB21M).

<div class='requirements' markdown="1">

Requires: A Duckiebot.

Requires: A stable network connection to your Duckiebot.

Result: A flashed microcontroller (not SD card) on the HUT board with the latest code.

</div>

Warning: You must not proceed with the following instructions unless you are in one of the following cases:
1. The motors of the robot are not working.
2. The LEDs are not white when the robot is on.
3. You know what you are doing, or you have been asked to flash the microcontroller by someone who knows what the consequences can be.

First of all, ssh into your Duckiebot running:

    laptop $ ssh duckie@DUCKIEBOT_NAME.local

Proceed to install the packages needed to compile the firmware:

    duckiebot $ sudo apt-get update
    duckiebot $ sudo apt-get install bison autoconf flex gcc-avr binutils-avr gdb-avr avr-libc avrdude build-essential

If you are running a Duckiebot with an NVIDIA Jetson Nano board, clone the firmware for the microcontroller using the following command:

    duckiebot $ git clone -b jetson-nano https://github.com/duckietown/fw-device-hut.git

else, if you have a Raspberry Pi based Duckiebot, use:

    duckiebot $ git clone -b raspberry-pi https://github.com/duckietown/fw-device-hut.git

Navigate inside the repository you cloned and copy the avrdude config file in the `/etc` folder of the Duckiebot:

    duckiebot $ cd fw-device-hut
    duckiebot $ sudo cp _avrdudeconfig/avrdude.conf /etc/avrdude.conf

Then test the avrdude and set the fuses (these are some low-level settings, which are set only once) with:

    duckiebot $ make fuses

If the command is successful, the connection to the microcontroller works, and the fuses are written. The output looks like this: 

    avrdude: verifying …
    avrdude: 1 bytes of efuse verified

    avrdude: safemode: Fuses OK (E:FF, H:DF, L:E2)


    avrdude done.  Thank you.

While in the case you see the `message make: warning: Clock skew detected. Your build may be incomplete.` or the process is not stopping, stop the process pressing <kbd>Ctrl</kbd>-<kbd>C</kbd> and run:

    duckiebot $ find -exec touch \{\} \;

After that, retry running the `make fuses` command.

Continue removing all temporary files, so everything has to be compiled freshly by running:

    duckiebot $ make clean

Then compile the firmware and upload it to the microcontroller by running:

    duckiebot $ make

The resulting output prints the following text:

    .....

    sudo avrdude -p attiny861 -c linuxgpio -P  -q -U flash:w:main.hex

    avrdude: AVR device initialized and ready to accept instructions

    Reading | ################################################## | 100% 0.00s

    avrdude: Device signature = 0x1e930d (probably t861)
    avrdude: NOTE: "flash" memory has been specified, an erase cycle will be performed
         To disable this feature, specify the -D option.
    avrdude: erasing chip
    avrdude: reading input file "main.hex"
    avrdude: input file main.hex auto detected as Intel Hex
    avrdude: writing flash (2220 bytes):

    Writing | ################################################## | 100% 0.75s

    avrdude: 2220 bytes of flash written
    avrdude: verifying flash memory against main.hex:
    avrdude: load data flash data from input file main.hex:
    avrdude: input file main.hex auto detected as Intel Hex
    avrdude: input file main.hex contains 2220 bytes
    avrdude: reading on-chip flash data:

    Reading | ################################################## | 100% 0.58s

    avrdude: verifying ...
    avrdude: 2220 bytes of flash verified

    avrdude: safemode: Fuses OK (E:FF, H:DF, L:E2)

    avrdude done.  Thank you.

Finally, remove the repository to free up space on the robot and reboot the Duckiebot:

    duckiebot $ cd .. && rm -rf fw-device-hut
    duckiebot $ sudo reboot


