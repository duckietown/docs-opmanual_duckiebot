# Debug - Reflash Microcontroller {#reflash-microcontroller status=ready}

<div class='requirements' markdown="1">

Requires: A Duckiebot of [configuration](#duckiebot-configurations) `DB18` or above.

Requires: A stable network connection to your Duckiebot.

Result: A flashed microcontroller (not SD card) on the HUT board, with the latest code version.

</div>

Warning: this procedure is needed for Duckietown HUT version `3.15`, but not for other models. Although often unnecessary, it is safe to perform on any HUT of version `2.0` and above.   

## When and why should I run this procedure? {#reflash-microcontroller-when}

This procedure flashes the microcontroller on the Duckietown HUT. This microcontroller is responsible for translating the duty cycle commands from the onboard computer to actual `PWM` signals that control the motors and the LEDs (because they are "addressable" LEDs) of the Duckiebots.

A typical example of when is necessary to flash the microcontroller is when commands are sent to the motors, e.g., through keyboard control, the motors signals on the dashboard/mission control show that signals are correctly being sent, but the Duckiebot does not move.  

This procedure will not be useful to fix problems such as one motor working and not the other, or LEDs showing unexpected colors when the motors work.

## How to flash the microcontroller {#reflash-microcontroller-how}

Ssh into your Duckiebot by running:

    laptop $ ssh duckie@![DUCKIEBOT_NAME].local


Install the packages needed to compile the microcontroller firmware:

    duckiebot $ sudo apt-get update
    duckiebot $ sudo apt-get install bison autoconf flex gcc-avr binutils-avr gdb-avr avr-libc avrdude build-essential

If you are running a Duckiebot with an NVIDIA Jetson Nano board, clone the firmware for the microcontroller using the following command:

    duckiebot $ git clone -b jetson-nano https://github.com/duckietown/fw-device-hut.git

else, if you have a Raspberry Pi based Duckiebot, use:

    duckiebot $ git clone -b raspberry-pi https://github.com/duckietown/fw-device-hut.git

Navigate inside the repository you cloned and copy the `avrdude config` file in the `/etc` folder of the Duckiebot:

    duckiebot $ cd fw-device-hut
    duckiebot $ sudo cp _avrdudeconfig/avrdude.conf /etc/avrdude.conf

Then test the `avrdude` and set the low-level configuration with:

    duckiebot $ make fuses

A successful outcome looks like:

    avrdude: verifying …
    avrdude: 1 bytes of efuse verified

    avrdude: safemode: Fuses OK (E:FF, H:DF, L:E2)


    avrdude done.  Thank you.

If you see the message `make: warning: Clock skew detected. Your build may be incomplete.` or the process is not stopping, stop the process pressing <kbd>Ctrl</kbd>-<kbd>C</kbd> and run:

    duckiebot $ find -exec touch \{\} \;

And then retry running the `make fuses` command.

Remove all temporary files by running:

    duckiebot $ make clean

Compile the firmware and upload it to the microcontroller:

    duckiebot $ make

The resulting output should be:

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

Remove the cloned repository to free up space:


    duckiebot $ cd .. && rm -rf fw-device-hut


and finally reboot the Duckiebot:

    duckiebot $ sudo reboot

After reboot your Duckiebot should move normally and LEDs respond nominally. The Dashboard / components page will show a green status for the HUT, too.