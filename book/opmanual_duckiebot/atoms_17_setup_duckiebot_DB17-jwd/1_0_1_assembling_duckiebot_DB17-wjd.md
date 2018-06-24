# Assembling the Duckiebot (`DB17-wjd`)  {#assembling-duckiebot-db17-ttic status=ready}

Point of contact: Andrea F. Daniele

Once you have received the parts and soldered the necessary components, it is time to assemble them in a Duckiebot. Here, we provide the assembly instructions for the configuration `DB17-wjd` (TTIC only).

<div class='requirements' markdown="1">

Requires: Duckiebot `DB17-wjd` parts. The acquisition process is explained in [](#acquiring-parts-c0).

Requires: Having soldered the `DB17-wjd` parts. The soldering process is explained in [](#soldering-boards-c0).

Requires: Having prepared the power cable. The power cable preparation is explained in [](#power-cable-prep-c0). Note: Not necessary if you intend to build a `DB17-l` configuration.

Requires: Time: about 30 minutes.

Results: An assembled Duckiebot in configuration `DB17-wjd`.

</div>

Note: The [FAQ](#op-assembly-db17wjd-faq) section at the bottom of this page may already answer some of you comments, questions or doubts.

This section is comprised of 14 parts. Each part builds upon some of the previous parts, so make
sure to follow them in the following order.

- [Part I: Motors](#howto-mount-motors)
- [Part II: Wheels](#howto-mount-wheels)
- [Part III: Omni-directional wheel](#howto-mount-omni-wheel)
- [Part IV: Chassis standoffs](#howto-mount-chassis-standoffs)
- [Part V: Camera kit](#howto-mount-camera-kit)
- [Part VI: Heat sinks](#howto-mount-heat-sinks)
- [Part VII: Raspberry Pi 3](#howto-mount-raspberry-pi-3)
- [Part VIII: Top plate](#howto-mount-top-plate)
- [Part IX: USB Power cable](#howto-prepare-usb-power-cable)
- [Part X: DC Stepper Motor HAT](#howto-mount-dc-stepper-motor-hat)
- [Part XI: Battery](#howto-mount-battery)
- [Part XII: Upgrade to `DB17-w`](#howto-upgrade-db17-w)
- [Part XIII: Upgrade to `DB17-j`](#howto-upgrade-db17-j)
- [Part XIV: Upgrade to `DB17-d`](#howto-upgrade-db17-d)


## Motors {#howto-mount-motors}

Open the Magician Chassis package ([](#fig:duckiebot_components)) and take out the following components:

- Chassis-bottom (1x)
- DC Motors (2x)
- Motor holders (4x)
- M3x30 screw (4x)
- M3 nuts (4x)

[](#fig:howto-mount-motors-parts) shows the components needed to complete this part of the tutorial.

<figure id="duckiebot_components" figure-caption="Components in Duckiebot package.">
     <img src="duckiebot_components.png" style='width: 30em'/>
</figure>

<div figure-id="fig:howto-mount-motors-parts" figure-caption="Components needed to mount the motors.">
     <img src="howto_mount_motors_parts.jpg" style='width: 30em'/>
</div>

### Video tutorial

The following video shows how to attach the motors to the bottom plate of the chassis.

<div figure-id="fig:howto-mount-motors-video">
    <dtvideo src="vimeo:236334401"/>
</div>



### Step-by-step guide

#### Step 1

Pass the motor holders through the openings in the bottom plate of the chassis as shown
in [](#fig:howto-mount-motors-sketch1).

<div figure-id="fig:howto-mount-motors-sketch1" figure-caption="The sketch of how to mount the motor holders.">
     <img src="howto_mount_motors_sketch1.jpg" style='width: 30em'/>
</div>


#### Step 2

Put the motors between the holders as shown in [](#fig:howto-mount-motors-sketch2).

<div figure-id="fig:howto-mount-motors-sketch2" figure-caption="The sketch of how to mount the motors.">
  <img src="howto_mount_motors_sketch2.jpg" style='width: 30em'/>
</div>

Note: Orient the motors so that their wires are inwards (i.e., towards the center of the plate).


#### Step 3

Use *4* M3x30 screws and *4* M3 nuts to secure the motors to the motor holders. Tighten the screws
to secure the holders to the bottom plate of the chassis as shown in [](#fig:howto-mount-motors-sketch3).

<div figure-id="fig:howto-mount-motors-sketch3" figure-caption="The sketch of how to secure the motors to the bottom plate.">
   <img src="howto_mount_motors_sketch3.jpg" style='width: 30em'/>
</div>


### Check the outcome

[](#fig:howto-mount-motors-milestone) shows how the motors should be attached to the bottom plate of the chassis.

<div figure-id="fig:howto-mount-motors-milestone" figure-caption="The motors are attached to the bottom plate of the chassis.">
   <img src="howto_mount_motors_milestone.jpg" style='width: 30em'/>
</div>



## Wheels {#howto-mount-wheels}

From the Magician Chassis package take the following components:

- Wheels (2x)

[](#fig:howto-mount-wheels-parts) shows the components needed to complete this part of the tutorial.

<div figure-id="fig:howto-mount-wheels-parts" figure-caption="The wheels.">
     <img src="howto_mount_wheels_parts.jpg" style='width: 30em'/>
</div>

### Video tutorial

The following video shows how to attach the wheels to the motors.

<div figure-id="fig:howto-mount-wheels-video">
    <dtvideo src="vimeo:236334418"/>
</div>

### Check the outcome

[](#fig:howto-mount-wheels-milestone) shows how the wheels should be attached to the motors.

<div figure-id="fig:howto-mount-wheels-milestone" figure-caption="The wheels are attached to the motors.">
   <img src="howto_mount_wheels_milestone.jpg" style='width: 30em'/>
</div>



## Omni-directional wheel {#howto-mount-omni-wheel}

The Duckiebot is driven by controlling the wheels attached to the DC motors.
Still, it requires a _passive_ support on the back. In this configuration an omni-directional wheel is
attached to the bottom plate of the chassis to provide such support.

From the Magician Chassis package take the following components:

- Steel omni-directional wheel (1x)
- Long metal spacers (2x)
- M3x6 screws (4x)

[](#fig:howto-mount-omni-wheel-parts) shows the components needed to complete this part of the tutorial.

<div figure-id="fig:howto-mount-omni-wheel-parts" figure-caption="The omni-directional wheel with *2* long
spacers and *4* M3x6 screws.">
     <img src="howto_mount_omni_wheel_parts.jpg" style='width: 30em'/>
</div>

### Video tutorial

The following video shows how to attach the omni-directional wheel to the bottom plate of the chassis.

<div figure-id="fig:howto-mount-omni-wheel-video">
    <dtvideo src="vimeo:236334422"/>
</div>

### Step-by-step guide

#### Step 1

Secure the long spacers to the plate using *2* M3x6 screws and the omni-directional wheel to the spacers
using also *2* M3x6 screws as shown in [](#fig:howto-mount-omni-wheel-sketch).

<div figure-id="fig:howto-mount-omni-wheel-sketch" figure-caption="The sketch of how to mount the omni-directional wheel.">
     <img src="howto_mount_omni_wheel_sketch.jpg" style='width: 30em'/>
</div>


### Check the outcome

[](#fig:howto-mount-omni-wheel-milestone) shows how the omni-directional wheel should be attached to the plate.

<div figure-id="fig:howto-mount-omni-wheel-milestone" figure-caption="The omni-directional wheel is attached to the plate.">
   <img src="howto_mount_omni_wheel_milestone.jpg" style='width: 30em'/>
</div>



## Chassis standoffs {#howto-mount-chassis-standoffs}

From the Magician Chassis package take the following components:

- Long metal spacers/standoffs (4x)
- M3x6 screws (4x)

From the Duckiebot kit take the following components:

- M3x5 nylon spacers/standoffs (4x)

[](#fig:howto-mount-standoffs-parts) shows the components needed to complete this part of the tutorial.

<div figure-id="fig:howto-mount-standoffs-parts" figure-caption="The standoffs to mount on the bottom plate.">
     <img src="howto_mount_standoffs_parts.jpg" style='width: 30em'/>
</div>

### Video tutorial

The following video shows how to attach the standoffs to the bottom plate of the chassis.

<div figure-id="fig:howto-mount-standoffs-video">
    <dtvideo src="vimeo:236334431"/>
</div>

### Step-by-step guide

#### Step 1

Secure the long metal spacers to the bottom plate using *4* M3x6 screws as shown in [](#fig:howto-mount-standoffs-sketch).

<div figure-id="fig:howto-mount-standoffs-sketch" figure-caption="The sketch of how to mount the standoffs on the plate.">
     <img src="howto_mount_standoffs_sketch.jpg" style='width: 30em'/>
</div>

#### Step 2

Attach the *4* nylon standoffs on top of the metal ones.


### Check the outcome

[](#fig:howto-mount-standoffs-milestone) shows how the standoffs should be attached to the plate.

<div figure-id="fig:howto-mount-standoffs-milestone" figure-caption="The standoffs attached to the plate.">
   <img src="howto_mount_standoffs_milestone.jpg" style='width: 30em'/>
</div>




## Camera kit {#howto-mount-camera-kit}

From the Magician Chassis package take the following components:

- M3x10 flathead screws (2x)
- M3 nuts (2x)

From the Duckiebot kit take the following components:

- Camera Module (1x)
- (Optional) 300mm Camera cable (1x)
- Camera mount (1x)

Note: If you have camera cables of different lengths available, keep in mind that both are going to work.
We suggest to use the longer one, and wrap the extra length under the Raspberry Pi stack.

[](#fig:howto-mount-camera-parts) shows the components needed to complete this part of the tutorial.

<div figure-id="fig:howto-mount-camera-parts" figure-caption="The parts needed to fix the camera on the top plate.">
     <img src="howto_mount_camera_parts.jpg" style='width: 30em'/>
</div>

### Video tutorial

The following video shows how to secure the camera to the top plate of the chassis.

<div figure-id="fig:howto-mount-camera-video">
    <dtvideo src="vimeo:236334448"/>
</div>

### Step-by-step guide

#### Step 1 (Optional)

If you do not have the 300mm Camera cable you can jump to *Step 3*.

If you do have the long camera cable, the first thing to do is removing the shorter cable that comes
attached to the camera module. Make sure to slide up the black connectors of the camera port on the camera
module in order to unblock the cable.

#### Step 2

Connect the camera cable to the camera module as shown in [](#fig:howto-mount-camera-cable).

<div figure-id="fig:howto-mount-camera-cable" figure-caption="How to connect the camera cable to the camera module.">
     <img src="howto_mount_camera_step1.jpg" style='width: 30em'/>
</div>

#### Step 3

Attach the camera module to the camera mount as shown in [](#fig:howto-mount-camera-mount).

<div figure-id="fig:howto-mount-camera-mount" figure-caption="How to attach the camera to the camera mount.">
     <img src="howto_mount_camera_step2.jpg" style='width: 30em'/>
</div>

Note: The camera is just press-fitted to the camera mount, no screws/nuts are needed.

#### Step 4

Secure the camera mount to the top plate by using the *2* M3x10 flathead screws and the nuts
as shown in [](#fig:howto-mount-camera-plate).

<div figure-id="fig:howto-mount-camera-plate" figure-caption="How to attach the camera mount to the top plate.">
     <img src="howto_mount_camera_step3.jpg" style='width: 30em'/>
</div>


### Check the outcome

[](#fig:howto-mount-camera-milestone) shows how the camera should be attached to the plate.

<div figure-id="fig:howto-mount-camera-milestone" figure-caption="The camera attached to the plate.">
   <img src="howto_mount_camera_milestone.jpg" style='width: 30em'/>
</div>



## Heat sinks {#howto-mount-heat-sinks}

From the Duckiebot kit take the following components:

- Raspberry Pi 3 (1x)
- Heat sinks (2x)
- Camera mount (1x)

[](#fig:howto-mount-heatsinks-parts) shows the components needed to complete this part of the tutorial.

<div figure-id="fig:howto-mount-heatsinks-parts" figure-caption="The heat sinks and the Raspberry Pi 3.">
     <img src="howto_mount_heatsinks_parts.jpg" style='width: 30em'/>
</div>

### Video tutorial

The following video shows how to install the heat sinks on the Raspberry Pi 3.

<div figure-id="fig:howto-mount-heatsinks-video">
    <dtvideo src="vimeo:236334458"/>
</div>

### Step-by-step guide

#### Step 1

Remove the protection layer from the heat sinks.

#### Step 2

Install the big heat sink on the big "Broadcom"-labeled integrated circuit (IC).

#### Step 3

Install the small heat sink on the small "SMSC"-labeled integrated circuit (IC).


### Check the outcome

[](#fig:howto-mount-heatsinks-milestone) shows how the heat sinks should be installed on the Raspberry Pi 3.

<div figure-id="fig:howto-mount-heatsinks-milestone" figure-caption="The heat sinks installed on the Raspberry Pi 3.">
   <img src="howto_mount_heatsinks_milestone.jpg" style='width: 30em'/>
</div>



## Raspberry Pi 3 {#howto-mount-raspberry-pi-3}

From the Magician Chassis package take the following components:

- Top plate (with camera attached) (1x)

From the Duckiebot kit take the following components:

- Raspberry Pi 3 (with heat sinks) (1x)
- M2.5x12 nylon spacers/standoffs (8x)
- M2.5 nylon hex nuts (4x)

[](#fig:howto-mount-rpi3-parts) shows the components needed to complete this part of the tutorial.

<div figure-id="fig:howto-mount-rpi3-parts" figure-caption="The parts needed to mount the Raspberry Pi 3 on the top plate.">
     <img src="howto_mount_rpi3_parts.jpg" style='width: 30em'/>
</div>

### Video tutorial

The following video shows how to mount the Raspberry Pi 3 on the top plate of the chassis.

<div figure-id="fig:howto-mount-rpi3-video">
    <dtvideo src="vimeo:236334461"/>
</div>

### Step-by-step guide

#### Step 1

Mount *8* M2.5x12 nylon standoffs on the Raspberry Pi 3 as shown in [](#fig:howto-mount-rpi3-standoffs).

<div figure-id="fig:howto-mount-rpi3-standoffs" figure-caption="How to mount the nylon standoffs on the Raspberry Pi 3.">
     <img src="howto_mount_rpi3_standoffs.jpg" style='width: 30em'/>
</div>

#### Step 2

Use the M2.5 nylon hex nuts to secure the Raspberry Pi 3 to the top plate as shown in [](#fig:howto-mount-rpi3-plate).

<div figure-id="fig:howto-mount-rpi3-plate" figure-caption="How to mount the Raspberry Pi 3 on the top plate.">
     <img src="howto_mount_rpi3_plate.jpg" style='width: 30em'/>
</div>


### Check the outcome

[](#fig:howto-mount-rpi3-milestone) shows how the Raspberry Pi 3 should be mounted on the top plate of the chassis.

<div figure-id="fig:howto-mount-rpi3-milestone" figure-caption="The Raspberry Pi 3 mounted on the top plate.">
   <img src="howto_mount_rpi3_milestone.jpg" style='width: 30em'/>
</div>



## Top plate {#howto-mount-top-plate}

From the Magician Chassis package take the following components:

- Bottom plate (with motors, wheels and standoffs attached) (1x)
- Top plate (with camera and Raspberry Pi 3 attached) (1x)
- M3x6 screws (4x)

[](#fig:howto-mount-plate-parts) shows the components needed to complete this part of the tutorial.

<div figure-id="fig:howto-mount-plate-parts" figure-caption="The parts needed to secure the top plate to the bottom plate.">
     <img src="howto_mount_plate_parts.jpg" style='width: 30em'/>
</div>

### Video tutorial

The following video shows how to secure the top plate on top of the bottom plate.

<div figure-id="fig:howto-mount-plate-video">
    <dtvideo src="vimeo:236334469"/>
</div>

### Step-by-step guide

#### Step 1

Pass the motor wires through the openings in the top plate.

#### Step 2

Use *4* M3x6 screws to secure the top plate to the nylon standoffs (mounted on the bottom plate in
[](#howto-mount-chassis-standoffs)) as shown in [](#fig:howto-mount-plate-sketch).

<div figure-id="fig:howto-mount-plate-sketch" figure-caption="How to secure the top plate to the bottom plate.">
     <img src="howto_mount_plate_sketch.jpg" style='width: 30em'/>
</div>


### Check the outcome

[](#fig:howto-mount-plate-milestone) shows how the top plate should be mounted on the bottom plate.

<div figure-id="fig:howto-mount-plate-milestone" figure-caption="The chassis completed.">
   <img src="howto_mount_plate_milestone.jpg" style='width: 30em'/>
</div>



## USB Power cable {#howto-prepare-usb-power-cable}

The power cable preparation is explained in [](#power-cable-prep-c0).



## DC Stepper Motor HAT {#howto-mount-dc-stepper-motor-hat}

From the Duckiebot kit take the following components:

- USB power cable (prepared in [](#power-cable-prep-c0)) (1x)
- DC Stepper Motor HAT (1x)
- M2.5x10 Nylon screws (or M2.5x12 nylon standoffs) (4x)

[](#fig:howto-mount-motorhat-parts) shows the components needed to complete this part of the tutorial.

<div figure-id="fig:howto-mount-motorhat-parts" figure-caption="The parts needed to add the DC Stepper Motor
HAT to the Duckiebot.">
     <img src="howto_mount_motorhat_parts.jpg" style='width: 30em'/>
</div>

### Video tutorial

The following video shows how to connect the DC Stepper Motor HAT to the Raspberry Pi 3.

<div figure-id="fig:howto-mount-motorhat-video">
    <dtvideo src="vimeo:236334482"/>
</div>

### Step-by-step guide

#### Step 1

Connect the wires of the USB power cable to the terminal block on the DC Stepper Motor HAT labeled as
"5-12V Motor Power" as shown in [](#fig:howto-mount-motorhat-cable).
The black wire goes to the negative terminal block (labeled with a minus: `-`) and the red wire goes to the
positive terminal block (labeled with a plus: `+`).

<div figure-id="fig:howto-mount-motorhat-cable" figure-caption="How to connect the USB power cable to
the DC Stepper Motor HAT.">
     <img src="howto_mount_motorhat_cable.jpg" style='width: 30em'/>
</div>

#### Step 2

Pass the free end of the camera cable through the opening in the DC Stepper Motor HAT as shown in [](#fig:howto-mount-motorhat-camera).

<div figure-id="fig:howto-mount-motorhat-camera" figure-caption="How to pass the camera cable through
the opening in the DC Stepper Motor HAT.">
     <img src="howto_mount_motorhat_camera.jpg" style='width: 30em'/>
</div>

#### Step 3

Connect the free end of the camera cable to the **CAMERA** port on the Raspberry Pi 3 as shown
in [](#fig:howto-mount-motorhat-connect-camera).

<div figure-id="fig:howto-mount-motorhat-connect-camera" figure-caption="How to connect the camera cable to
the CAMERA port on the Raspberry Pi 3.">
    <img src="howto_mount_motorhat_connect_camera.jpg" style='width: 30em'/>
</div>

To do so, you will need to gently pull up on the black connector (it will slide up) to allow the cable to
insert the port. Slide the connector back down to lock the cable in place, making sure it “clicks”.

Note: Make sure the camera cable is inserted in the right direction! The metal pins of the cable must
be in contact with the metal terminals in the camera port of the PI. Please be aware that different camera
cables have the text on different sides and with different orientation, **do not** use it as a landmark.

#### Step 4

Attach the DC Stepper Motor HAT to the GPIO header on the Raspberry Pi 3.
Make sure that the GPIO stacking header of the Motor HAT is carefully aligned with the underlying GPIO pins
before applying pressure.

Note: In case you are using a short camera cable, ensure that the camera cable does not stand between the
GPIO pins and the the GPIO header socket before applying pressure.

#### Step 5

Secure the DC Stepper Motor HAT using *4* M2.5x10 nylon screws.

Note: If you are planning on upgrading your Duckiebot to the configuration `DB17-l`, you can use *4* M2.5x12
nylon standoffs instead.

#### Step 6

Connect the motor wires to the terminal block on the DC Stepper Motor HAT as shown
in [](#fig:howto-mount-motorhat-motors).

<div figure-id="fig:howto-mount-motorhat-motors" figure-caption="How to connect the motor wires to the
terminal block on the DC Stepper Motor HAT.">
     <img src="howto_mount_motorhat_motors.jpg" style='width: 30em'/>
</div>

While looking at the Duckiebot from the back, identify the wires for left and right motor. Connect the
left motor wires to the terminals labeled as **M1** and the right motor wires to the terminals labeled
as **M2**. This will ensure that the pre-existing software that we will later install on the Duckiebot
will send the commands to the correct motors.

### Check the outcome

[](#fig:howto-mount-motorhat-milestone) shows how the DC Stepper Motor HAT should be connected to the
Raspberry Pi 3.

<div figure-id="fig:howto-mount-motorhat-milestone" figure-caption="The DC Stepper Motor HAT connected to the Raspberry Pi 3.">
   <img src="howto_mount_motorhat_milestone.jpg" style='width: 30em'/>
</div>



## Battery {#howto-mount-battery}

From the Duckiebot kit take the following components:

- Battery (1x)
- Zip tie (1x)
- Short micro USB cable (1x)

[](#fig:howto-mount-battery-parts) shows the components needed to complete this part of the tutorial.

<div figure-id="fig:howto-mount-battery-parts" figure-caption="The parts needed to add the battery to the Duckiebot.">
     <img src="howto_mount_battery_parts.jpg" style='width: 30em'/>
</div>

### Video tutorial

The following video shows how to add the battery to the Duckiebot and turn it on.

<div figure-id="fig:howto-mount-battery-video">
    <dtvideo src="vimeo:236334493"/>
</div>

### Step-by-step guide

#### Step 1

Pass the zip tie through the opening in the top plate.

#### Step 2

Slide the battery between the two plates. Make sure it is above the zip tie.

#### Step 3

Push the free end of the zip tie through the opening in the top plate.

#### Step 4

Tighten the zip tie to secure the battery.

#### Step 5

Connect the short micro USB cable to the Raspberry Pi 3.

#### Step 6

Connect the short micro USB cable to the battery.

#### Step 7

Connect the USB power cable to the battery.

#### Step 8

Make sure that the LEDs on the Raspberry Pi 3 and the DC Stepper Motor HAT are on.

### Check the outcome

[](#fig:howto-mount-battery-milestone) shows how the battery should be installed on the Duckiebot.

<div figure-id="fig:howto-mount-battery-milestone" figure-caption="The configuration `DB17` completed.">
   <img src="howto_mount_battery_milestone.jpg" style='width: 30em'/>
</div>



## Upgrade to `DB17-w` {#howto-upgrade-db17-w}

This upgrade equips the Duckiebot with a secondary, faster, Wi-Fi connection, ideal for image streaming.
The new configuration is called `DB17-w`.

[](#fig:howto-mount-db17w-parts) shows the components needed to complete this upgrade.

<div figure-id="fig:howto-mount-db17w-parts" figure-caption="The parts needed to upgrade the
Duckiebot to the configuration DB17-w.">
     <img src="howto_upgrade_db17w_parts.jpg" style='width: 30em'/>
</div>

### Instructions

- Insert the USB WiFi dongle into one of the USB ports of the Raspberry Pi.

<div figure-id="fig:howto-mount-db17w-milestone" figure-caption="Upgrade to DB17-w completed.">
     <img src="howto_upgrade_db17w_milestone.jpg" style='width: 30em'/>
</div>



## Upgrade to `DB17-j` {#howto-upgrade-db17-j}

This upgrade equips the Duckiebot with manual remote control capabilities. It is particularly
useful for getting the Duckiebot out of tight spots or letting younger ones have a drive, in
addition to providing handy shortcuts to different functions in development phase.
The new configuration is called `DB17-j`.

[](#fig:howto-mount-db17j-parts) shows the components needed to complete this upgrade.

<div figure-id="fig:howto-mount-db17j-parts" figure-caption="The parts needed to upgrade the
Duckiebot to the configuration DB17-j.">
     <img src="howto_upgrade_db17j_parts.jpg" style='width: 30em'/>
</div>

Note: The joystick comes with a USB receiver (as shown in [](#fig:howto-mount-db17j-parts)).

### Instructions

- Insert the USB receiver into one of the USB ports of the Raspberry Pi.
- Insert 2 AA batteries on the back side of the joystick.
- Turn on the joystick by pressing the `HOME` button. Make sure that the LED above the `SELECT` button is steady.

<div figure-id="fig:howto-mount-db17j-milestone" figure-caption="Upgrade to DB17-j completed.">
     <img src="howto_upgrade_db17j_milestone.jpg" style='width: 30em'/>
</div>

TODO: explain how to test the joystick with `jstest`



## Upgrade to `DB17-d` {#howto-upgrade-db17-d}

This upgrade equips the Duckiebot with an external hard drive that is convenient for storing
videos (logs) as it provides both extra capacity and faster data transfer rates than the microSD
card in the Raspberry Pi 3. Moreover, it is easy to unplug it from the Duckiebot at the end of the
day and bring it over to a computer for downloading and analyzing stored data.
The new configuration is called `DB17-d`.

[](#fig:howto-mount-db17d-parts) shows the components needed to complete this upgrade.

<div figure-id="fig:howto-mount-db17d-parts" figure-caption="The parts needed to upgrade the
Duckiebot to the configuration DB17-d.">
     <img src="howto_upgrade_db17d_parts.jpg" style='width: 30em'/>
</div>

### Instructions

- Insert the USB drive into one of the USB ports of the Raspberry Pi.

<div figure-id="fig:howto-mount-db17d-milestone" figure-caption="Upgrade to DB17-d completed.">
     <img src="howto_upgrade_db17d_milestone.jpg" style='width: 30em'/>
</div>

- Mount your USB drive as explained in [](+software_reference#mounting-usb).



## FAQ {#op-assembly-db17wjd-faq}

Q: If we have the bumpers, at what point should we add them?

A: You shouldn't have the bumpers at this point. The function of the bumpers is to keep the LEDs in place, i.e., they belong to `DB17-l` configuration. These instructions cover the `DB17-wjd` configurations. You will find the bumper assembly instructions in [](+opmanual_duckiebot##assembling-duckiebot-c1).

Q: Yeah but I still have the bumpers and am reading this page. So?

A: The bumpers can be added after the Duckiebot assembly is complete.

Q: I found it hard to mount the camera (the holes weren't lining up).

A: Sometimes in life you have to push a little to make things happen. (But don't push too much or things will break!)

Q: The long camera cable is a bit annoying - I folded it and shoved it in between two hats.

A: The shorter cable is even more annoying. We suggest wrapping the long camera cable between the chassis and the Raspberry Pi. With some strategic planning, you can use the zipties that keep the battery in place to hold the camera cable in place as well ([see figure below-to add]())

TODO: add pretty cable handling pic

Q: I found that the screwdriver that comes with the chassis kit is too fat to screw in the wires on the hat.

A: It is possible you got one of the fatter screwdrivers. You will need to figure it out yourself (or ask a TA for help).

Q: I need something to cut the end of the zip tie with.

A: Scissors typically work out for these kind of jobs (and no, they're not provided in a Fall 2017 Duckiebox).
