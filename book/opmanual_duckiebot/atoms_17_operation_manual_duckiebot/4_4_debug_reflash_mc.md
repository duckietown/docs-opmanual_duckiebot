# Debug - Reflash Microcontroller {#reflash-microcontroller status=ready}

Warning: You do not need to perform the following procedure unless you are specifically told to do in the book.

The LEDs of the Duckiebot should light up in white as soon as you power the Duckiebot. If the LEDs turn on and shine in any different color than white, probably the code on the microcontroller is corrupted. You can reflash it using the following procedure:

`ssh` into your robot and clone the Duckietown Software repository with:

    duckiebot $ git clone https://github.com/duckietown/Software.git ~/Software

install avrdude and gcc with:

    duckiebot $ sudo apt-get update
    duckiebot $ sudo apt-get install bison autoconf flex
    duckiebot $ sudo apt-get install gcc-avr binutils-avr gdb-avr avr-libc avrdude
    duckiebot $ sudo apt-get install build-essential

Copy the avrdude config file with:

    duckiebot $ cd ~/Software/hardware/software/_avrdudeconfig
    duckiebot $ sudo cp avrdude.conf /etc/avrdude.conf

Test avrdude and set fuses with:

    duckiebot $ cd ~/Software/hardware/software
    duckiebot $ make fuses

if the output of `make fuses` is at the end like

    avrdude: verifying …
    avrdude: 1 bytes of efuse verified

    avrdude: safemode: Fuses OK (E:FF, H:DF, L:E2)


    avrdude done.  Thank you.

the connection to the microcontroller works and the fuses could be written. The fuses are some low lowlevel settings, which just have to be set once. If this succeeded, jump the next step.

If there is the message "make: warning: Clock skew detected. Your build may be incomplete." or the make process is not stopping and many debugging messages are showed, try the following:
Press <kbd>Ctrl</kbd>-<kbd>C</kbd> to stop the current command. Then run:

    duckiebot $ find -exec touch \{\} \;

This ensures that the modification time of all files is set to the current time. Make decides, which files have to be compiled by comparing the source file time with the executable file time. If the executable file time lies in the future regarding the current system time, not all modified files are compiled. This could happen when the clock of the Raspberry Pi is not set correctly and the file timestamps of, e.g., a github pull are used.

Next up, remove all temporary files, so everything has to be compiled freshly by running:

    duckiebot $ make clean

Compile the programm and download it to the microcontroller by running:

    duckiebot $ make

the output should look like:

    Errors: none
    -------- end --------

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

With that, the microcontroller should work. To change the microcontroller programm, just edit the files, e.g with vim. With `make` you can compile and download the programm to the microcontroller again.

In the end, make sure to delete the Software repository to free up the space again. This is done by running:

    duckiebot $ rm -rf ~/Software
