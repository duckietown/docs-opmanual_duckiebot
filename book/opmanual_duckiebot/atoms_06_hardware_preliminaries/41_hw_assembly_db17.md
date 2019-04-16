# Assembling the `DB17` Duckiebot {#assembling-duckiebot-db17 status=ready}

This page is for the `DB17` configuration used in classes in 2017. The docker based software stack of 2018 is currently not guaranteed to work out of the box with the `DB17` hardware configurations.

<div class='requirements' markdown="1">

Requires: Duckiebot `DB17` parts. The acquisition process is explained in [](#get-db17-hw).

Requires: A microSD card with the Duckiebot image already on it. This procedure is explained [here](#setup-duckiebot).

Requires: Time: about 1-1.5 hours (45 minutes for an experienced Duckiebot builder).

Results: An assembled Duckiebot in configuration `DB17`.

</div>

There are several steps in this procedure. Many steps rely on previous ones, so make sure you perform them in order!

- [Part 1: Assemble the `DB17-jwd`](#assembling-duckiebot-db17jwd)
- [Part 2: Solder the boards](#assembling-duckiebot-db17jwd)
- [Part 3: Prepare the power cable for `DB17-jwd`](#assembling-duckiebot-db17-cable-splitting)
- [Part 4: Assemble the bumpers for `DB17-l`](#bumper-assembly)
- [Part 5: Assemble the `DB17-l`](#assembling-duckiebot-db17l)


## Assembly instructions (`DB17-jwd`) {#assembling-duckiebot-db17jwd status=ready}

Once you have received the parts and soldered the necessary components, it is time to assemble them in a Duckiebot. Here, we provide the assembly instructions for the configuration `DB17-wjd`.

<div class='requirements' markdown="1">

Requires: Duckiebot `DB17-wjd` parts. The acquisition process is explained in [](#db-opmanual-get-db17).

Requires: Having soldered the `DB17-wjd` parts. The soldering process is explained in [](#assembling-duckiebot-db17-soldering).

Requires: Having prepared the power cable. The power cable preparation is explained in [](#assembling-duckiebot-db17-cable-splitting). Note: Not necessary if you intend to build a `DB17-l` configuration.

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


### Motors {#howto-mount-motors}

Open the Magician Chassis package and take out the following components:

- Chassis-bottom (1x)
- DC Motors (2x)
- Motor holders (4x)
- M3x30 screw (4x)
- M3 nuts (4x)

[](#fig:howto-mount-motors-parts) shows the components needed to complete this part of the tutorial.

<div figure-id="fig:howto-mount-motors-parts" figure-caption="Components needed to mount the motors.">
     <img src="howto_mount_motors_parts.jpg" style='width: 30em'/>
</div>

#### Video tutorial

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


#### Check the outcome

[](#fig:howto-mount-motors-milestone) shows how the motors should be attached to the bottom plate of the chassis.

<div figure-id="fig:howto-mount-motors-milestone" figure-caption="The motors are attached to the bottom plate of the chassis.">
   <img src="howto_mount_motors_milestone.jpg" style='width: 30em'/>
</div>



### Wheels {#howto-mount-wheels}

From the Magician Chassis package take the following components:

- Wheels (2x)

[](#fig:howto-mount-wheels-parts) shows the components needed to complete this part of the tutorial.

<div figure-id="fig:howto-mount-wheels-parts" figure-caption="The wheels.">
     <img src="howto_mount_wheels_parts.jpg" style='width: 30em'/>
</div>

#### Video tutorial

The following video shows how to attach the wheels to the motors.

<div figure-id="fig:howto-mount-wheels-video">
    <dtvideo src="vimeo:236334418"/>
</div>

#### Check the outcome

[](#fig:howto-mount-wheels-milestone) shows how the wheels should be attached to the motors.

<div figure-id="fig:howto-mount-wheels-milestone" figure-caption="The wheels are attached to the motors.">
   <img src="howto_mount_wheels_milestone.jpg" style='width: 30em'/>
</div>



### Omni-directional wheel {#howto-mount-omni-wheel}

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

#### Video tutorial

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


#### Check the outcome

[](#fig:howto-mount-omni-wheel-milestone) shows how the omni-directional wheel should be attached to the plate.

<div figure-id="fig:howto-mount-omni-wheel-milestone" figure-caption="The omni-directional wheel is attached to the plate.">
   <img src="howto_mount_omni_wheel_milestone.jpg" style='width: 30em'/>
</div>



### Chassis standoffs {#howto-mount-chassis-standoffs}

From the Magician Chassis package take the following components:

- Long metal spacers/standoffs (4x)
- M3x6 screws (4x)

From the Duckiebot kit take the following components:

- M3x5 nylon spacers/standoffs (4x)

[](#fig:howto-mount-standoffs-parts) shows the components needed to complete this part of the tutorial.

<div figure-id="fig:howto-mount-standoffs-parts" figure-caption="The standoffs to mount on the bottom plate.">
     <img src="howto_mount_standoffs_parts.jpg" style='width: 30em'/>
</div>

#### Video tutorial

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


#### Check the outcome

[](#fig:howto-mount-standoffs-milestone) shows how the standoffs should be attached to the plate.

<div figure-id="fig:howto-mount-standoffs-milestone" figure-caption="The standoffs attached to the plate.">
   <img src="howto_mount_standoffs_milestone.jpg" style='width: 30em'/>
</div>

### Camera kit {#howto-mount-camera-kit}

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

#### Video tutorial

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

#### Check the outcome

[](#fig:howto-mount-camera-milestone) shows how the camera should be attached to the plate.

<div figure-id="fig:howto-mount-camera-milestone" figure-caption="The camera attached to the plate.">
   <img src="howto_mount_camera_milestone.jpg" style='width: 30em'/>
</div>

### Heat sinks {#howto-mount-heat-sinks}

From the Duckiebot kit take the following components:

- Raspberry Pi 3 (1x)
- Heat sinks (2x)
- Camera mount (1x)

[](#fig:howto-mount-heatsinks-parts) shows the components needed to complete this part of the tutorial.

<div figure-id="fig:howto-mount-heatsinks-parts" figure-caption="The heat sinks and the Raspberry Pi 3.">
     <img src="howto_mount_heatsinks_parts.jpg" style='width: 30em'/>
</div>

#### Video tutorial

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


#### Check the outcome

[](#fig:howto-mount-heatsinks-milestone) shows how the heat sinks should be installed on the Raspberry Pi 3.

<div figure-id="fig:howto-mount-heatsinks-milestone" figure-caption="The heat sinks installed on the Raspberry Pi 3.">
   <img src="howto_mount_heatsinks_milestone.jpg" style='width: 30em'/>
</div>

### Raspberry Pi 3 {#howto-mount-raspberry-pi-3}

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

#### Video tutorial

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


#### Check the outcome

[](#fig:howto-mount-rpi3-milestone) shows how the Raspberry Pi 3 should be mounted on the top plate of the chassis.

<div figure-id="fig:howto-mount-rpi3-milestone" figure-caption="The Raspberry Pi 3 mounted on the top plate.">
   <img src="howto_mount_rpi3_milestone.jpg" style='width: 30em'/>
</div>



### Top plate {#howto-mount-top-plate}

From the Magician Chassis package take the following components:

- Bottom plate (with motors, wheels and standoffs attached) (1x)
- Top plate (with camera and Raspberry Pi 3 attached) (1x)
- M3x6 screws (4x)

[](#fig:howto-mount-plate-parts) shows the components needed to complete this part of the tutorial.

<div figure-id="fig:howto-mount-plate-parts" figure-caption="The parts needed to secure the top plate to the bottom plate.">
     <img src="howto_mount_plate_parts.jpg" style='width: 30em'/>
</div>

#### Video tutorial

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


#### Check the outcome

[](#fig:howto-mount-plate-milestone) shows how the top plate should be mounted on the bottom plate.

<div figure-id="fig:howto-mount-plate-milestone" figure-caption="The chassis completed.">
   <img src="howto_mount_plate_milestone.jpg" style='width: 30em'/>
</div>



### USB Power cable {#howto-prepare-usb-power-cable}

The power cable preparation is explained in [](#assembling-duckiebot-db17-cable-splitting).



### DC Stepper Motor HAT {#howto-mount-dc-stepper-motor-hat}

From the Duckiebot kit take the following components:

- USB power cable (prepared in [](#assembling-duckiebot-db17-cable-splitting)) (1x)
- DC Stepper Motor HAT (1x)
- M2.5x10 Nylon screws (or M2.5x12 nylon standoffs) (4x)

[](#fig:howto-mount-motorhat-parts) shows the components needed to complete this part of the tutorial.

<div figure-id="fig:howto-mount-motorhat-parts" figure-caption="The parts needed to add the DC Stepper Motor
HAT to the Duckiebot.">
     <img src="howto_mount_motorhat_parts.jpg" style='width: 30em'/>
</div>

#### Video tutorial

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

#### Check the outcome

[](#fig:howto-mount-motorhat-milestone) shows how the DC Stepper Motor HAT should be connected to the
Raspberry Pi 3.

<div figure-id="fig:howto-mount-motorhat-milestone" figure-caption="The DC Stepper Motor HAT connected to the Raspberry Pi 3.">
   <img src="howto_mount_motorhat_milestone.jpg" style='width: 30em'/>
</div>



### Battery {#howto-mount-battery}

From the Duckiebot kit take the following components:

- Battery (1x)
- Zip tie (1x)
- Short micro USB cable (1x)

[](#fig:howto-mount-battery-parts) shows the components needed to complete this part of the tutorial.

<div figure-id="fig:howto-mount-battery-parts" figure-caption="The parts needed to add the battery to the Duckiebot.">
     <img src="howto_mount_battery_parts.jpg" style='width: 30em'/>
</div>

#### Video tutorial

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

#### Check the outcome

[](#fig:howto-mount-battery-milestone) shows how the battery should be installed on the Duckiebot.

<div figure-id="fig:howto-mount-battery-milestone" figure-caption="The configuration `DB17` completed.">
   <img src="howto_mount_battery_milestone.jpg" style='width: 30em'/>
</div>



### Upgrade to `DB17-w` {#howto-upgrade-db17-w}

This upgrade equips the Duckiebot with a secondary, faster, Wi-Fi connection, ideal for image streaming.
The new configuration is called `DB17-w`.

[](#fig:howto-mount-db17w-parts) shows the components needed to complete this upgrade.

<div figure-id="fig:howto-mount-db17w-parts" figure-caption="The parts needed to upgrade the
Duckiebot to the configuration DB17-w.">
     <img src="howto_upgrade_db17w_parts.jpg" style='width: 30em'/>
</div>

#### Instructions

- Insert the USB WiFi dongle into one of the USB ports of the Raspberry Pi.

<div figure-id="fig:howto-mount-db17w-milestone" figure-caption="Upgrade to DB17-w completed.">
     <img src="howto_upgrade_db17w_milestone.jpg" style='width: 30em'/>
</div>



### Upgrade to `DB17-j` {#howto-upgrade-db17-j}

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

#### Instructions

- Insert the USB receiver into one of the USB ports of the Raspberry Pi.
- Insert 2 AA batteries on the back side of the joystick.
- Turn on the joystick by pressing the `HOME` button. Make sure that the LED above the `SELECT` button is steady.

<div figure-id="fig:howto-mount-db17j-milestone" figure-caption="Upgrade to DB17-j completed.">
     <img src="howto_upgrade_db17j_milestone.jpg" style='width: 30em'/>
</div>

### Upgrade to `DB17-d` {#howto-upgrade-db17-d}

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

#### Instructions

- Insert the USB drive into one of the USB ports of the Raspberry Pi.

<div figure-id="fig:howto-mount-db17d-milestone" figure-caption="Upgrade to DB17-d completed.">
     <img src="howto_upgrade_db17d_milestone.jpg" style='width: 30em'/>
</div>

- Mount your USB drive.

TODO: re-add instructions from 2017 Duckiebook version.

### FAQ {#op-assembly-db17wjd-faq}

Q: If we have the bumpers, at what point should we add them?

A: You shouldn't have the bumpers at this point. The function of the bumpers is to keep the LEDs in place, i.e., they belong to `DB17-l` configuration. These instructions cover the `DB17-wjd` configurations. You will find the bumper assembly instructions in [](#bumper-assembly).

Q: Yeah but I still have the bumpers and am reading this page. So?

A: The bumpers can be added after the Duckiebot assembly is complete.

Q: I found it hard to mount the camera (the holes weren't lining up).

A: Sometimes in life you have to push a little to make things happen. (But don't push too much or things will break!)

Q: The long camera cable is a bit annoying - I folded it and shoved it in between two hats.

A: The shorter cable is even more annoying. We suggest wrapping the long camera cable between the chassis and the Raspberry Pi. With some strategic planning, you can use the zipties that keep the battery in place to hold the camera cable in place as well.

Q: I need something to cut the end of the zip tie with.

A: Scissors typically work out for these kind of jobs (and no, they're not provided in a Fall 2017 Duckiebox).

## Assembly instructions (`DB17`): soldering {#assembling-duckiebot-db17-soldering status=ready}

Note: It is better to be safe than sorry. Soldering is a potentially hazardous activity. There is a fire hazard as well as the risk of inhaling toxic fumes. Stop a second and make sure you are addressing the safety standards for soldering when following these instructions. If you have never soldered before, seek advice.

In this instruction set we will assume you have soldered something before and are acquainted with the soldering fundamentals. If not, before proceeding, read this great tutorial on soldering:

See: [Alternative instructions: how to solder on Headers and Terminal Block](https://learn.adafruit.com/adafruit-dc-and-stepper-motor-hat-for-raspberry-pi/assembly)

Note: Very general tips in soldering

* solder the components according to their height - from lowest to highest.

* soldering is potentially dangerous - prepare a clean, well lit and aerated working place before starting.

* It's ok to ask for help! Especially is you have never soldered before.

### Assembly instructions (`DB17-jwd`): DC Motor HAT

<div class='requirements' markdown="1">

Requires: Parts: Duckiebot `DB17` parts. The acquisition process is explained in [](#get-db17-hw). The configurations are described in [](#duckiebot-configurations). In particular you need:

- [GPIO Stacking Header](http://adafru.it/2223)
- [DC and Stepper Motor HAT for Raspberry Pi](http://adafru.it/2348)
- [Soldering tools](http://tinyurl.com/yyusy73b)

Requires: Experience: novice-level experience with soldering.

Requires: Time: 20 minutes

Results: Soldered DC Motor HAT

</div>

#### Preparing the components

Take the GPIO stacking header [](#fig:GPIO_Stacking_Header) out of Duckiebox and sort the following components from DC motor HAT package:

- Adafruit DC/Stepper Motor HAT for Raspberry Pi

- 2-pin terminal block (2x), 3-pin terminal block (1x)

<div figure-id="fig:GPIO_Stacking_Header" figure-caption="GPIO_Stacking_Header">
     <img src="GPIO_Stacking_Header.jpg" style='width: 30ex; height: auto'/>
</div>

<div figure-id="fig:DC/Stepper_HAT" figure-caption="DC/Stepper Motor HAT and solder components">
    <img src="DC_stepper_HAT.jpg" style='width: 30ex; height: auto'/>
</div>

#### Soldering instructions

1) Make a 5 pin terminal block by sliding the included 2 pin and 3 pin terminal blocks into each other [](#fig:terminal_block).

<div figure-id="fig:terminal_block" figure-caption="5 pin terminal_block">
   <img src="terminal_block.jpg" style='width: 30ex; height: auto'/>
</div>

2) Slide this 5 pin block through the holes just under "M1 GND M2" on the board. Solder it on (we only use two motors and do not need connect anything at the "M3 GND M4" location) ([](#figure:upview_Stepper_Motor));

3) Slide a 2 pin terminal block into the corner for power. Solder it on. ([](#figure:sideview_terminal));

4) Slide in the GPIO Stacking Header onto the 2x20 grid of holes on the edge opposite the terminal blocks and with vice versa direction ([](#figure:GPIO_HAT_orientation)). Solder it on.

Note: stick the GPIO Stacking Header from bottom to top, different orientation than terminal blocks (from top to bottom).


<div figure-id="fig:GPIO_HAT_orientation" figure-caption=" ">
   <img src="GPIO_HAT_orientation.jpg" style='width: 30ex; height: auto'/>
</div>

<div figure-id="fig:sideview_terminal" figure-caption="Side view of finished soldering DC/Stepper Motor HAT">
   <img src="sideview_Stepper_HAT.jpg" style='width: 30ex; height: auto'/>
</div>

<div figure-id="fig:upview_Stepper_Motor" figure-caption="upside view of finished soldering DC/Stepper Motor HAT">
   <img src="upview_stepper_Motor.jpg" style='width: 30ex; height: auto'/>
</div>

### Assembly instructions (`DB17-l`): soldering the PWM / Servo HAT and LED boards

<div class='requirements' markdown="1">

Requires: Duckiebot `DB17-l` parts. The acquisition process is explained in [](#get-db17-hw).
The configurations are described in [](#duckiebot-configurations). In particular you need:

- [GPIO Stacking Header](http://adafru.it/2223)
- [Adafruit PWM / Servo HAT](http://adafru.it/2327)
- [LED HAT](https://tinyurl.com/ydh9wqp5)

Requires: Time: 30 minutes

Results: A soldered PWM / Servo HAT for `DB17-l` configuration.

</div>

#### 16-channel PWM/Servo HAT

([Alternative instructions: how to solder on the PWM/Servo HAT](https://learn.adafruit.com/adafruit-16-channel-pwm-servo-hat-for-raspberry-pi/))


#### Prepare the components

Put the following components on the table according the Figure

* [GPIO Stacking Header](http://adafru.it/2223) for A+/B+/Pi 2
* [Adafruit HAT](http://adafru.it/2327) Mini Kit of 16-Channel PWM / Servo HAT for Raspberry Pi
    * 3x4 headers (4x)
    * 2-pin terminal block
    * 16-Channel PWM / Servo HAT for Raspberry Pi (1x)
* [LED HAT](https://tinyurl.com/ydh9wqp5)

<div figure-id="fig: " figure-caption=" ">
     <img src="image_3.jpg" style='width: 20ex; height: auto'/>
</div>

### PWM / Servo HAT soldering instructions

1. Solder the 2 pin terminal block next to the power cable jack
2. Solder the four 3x4 headers onto the edge of the HAT, below the words "Servo/PWM Pi HAT!"
3. Solder the GPIO Stacking Header at the top of the board, where the 2x20 grid of holes is located.

### LSD board soldering instructions

The LSD board you received is unpopulated and should look like this:

<img src="image_5.png" style='width: 20em; height: auto'/>

<img src="image_6.png" style='width: 20em; height: auto'/>

#### Prepare the components

Put the following components according the figure on the table:

* 1 x 40 pin female header
* 5 x 4 pin female header
* 2 x 16 pin male header
* 1 x 12 pin male header
* 1 x 3 pin male header
* 1 x 2 pin female shunt jumper
* 5 x 200 Ohm resistors
* 10 x 130 Ohm resistors
* 3 x 4 pin male header for servos

<div figure-id="fig: LSD_HAT_and_all_components" figure-caption="LSD HAT and all of needed components">
     <img src="LSD_HAT.jpg" style='width: 20em; height: auto'/>
</div>

#### Soldering instructions

1. Put the resistors on the top of the board according to silkscreen markings, solder it on from the bottom side.

Tips:

2. Solder all female headers to the bottom of the board. Alignment becomes easy if the  female headers are plugged into the PWM heat, and the LSD board rests on top.

3. Solder all male headers to the top of the board. Male header positions are outlined on the silkscreen.


#### LED connection

<img src="image_11.jpg" style='width: 20ex; height: auto'/>

<img src="image_12.jpg" style='width: 20ex; height: auto'/>


Parts list:

* 4 x 6" female-female jumper cable

Instructions:

1. Connect LED accordingly to silkscreen indication on PRi 2 LSD board

2. silkscreen legend: Rx, Gx, Bx are red, green, and blue channels, accordingly, where x is the LED number; C is a common line (either common anode or common cathode)

3. For adafruit LEDs are common anode type. The longest pin is common anode. Single pin on the side of common is red channel. The two other pins are Green and Blue channels, with the blue furthest from the common pin.

4. Both types of LEDs are supported. Use shunt jumper to select either common anode (CA) or common cathode (CC) on 3-pin male header. Note, however, that all LEDs on the board must be of the same type.


#### Putting things together

1. Stack the boards

    * Screw the first eight standoffs into the Pi - provide hints on the location of standoffs and the suggested orientation of the boards w/r to the chassis

    * connect the camera to the Pi [image showing the connector ?]

    * Stack the DC/Stepper Motor HAT onto the Pi, aligning both sets of GPIO pins over each other and screw the standoffs to secure it. Try to not bend the camera connector too much during this step

    * Stack the 16-channel PWM/Servo HAT onto the Pi, both sets of GPIO pins over each other and screw the standoffs to secure it

2. Slide the battery between the two chassis plates

3. Power the PWM/Servo HAT and Pi connecting them to the battery with the cables included in the duckiebox

4. Power the DC/Stepper motor from the PWM/Servo HAT using the male-to-male cable in the duckiebox, connect the positive

5. connect the Pi to the board

6. Done!

### Assembly instructions (`DB17-jwd`): power cable {#assembling-duckiebot-db17-cable-splitting status=ready}

In configuration `DB17` we will need a cable to power the DC motor HAT from the battery. The keen observer might have noticed that such a cable was not included in the [`DB17` Duckiebot parts](#db-opmanual-get-db17) chapter. Here, we create this cable by splitting open any USB-A cable, identifying and stripping the power wires, and using them to power the DC motor HAT. If you are unsure about the definitions of the different Duckiebot configurations, read [](#duckiebot-configurations).

It is important to note that these instructions are relevant only for assembling a `DB17-wjdc` configuration Duckiebot (or any subset of it). If you intend to build a `DB17-l` configuration Duckiebot, you can skip these instructions.

<div class='requirements' markdown="1">

Requires: One male USB-A to anything cable.

Requires: A pair of scissors.

Requires: A multimeter (only if you are not purchasing the [suggested components](#db-opmanual-get-db17))

Requires: Time: 5 minutes

Results: One male USB-A to wires power cable

</div>

#### Video tutorial

The following video shows how to prepare the USB power cable for the configuration `DB17`.

<div figure-id="fig:prepare-usb-cable">
    <dtvideo src="vimeo:236334476"/>
</div>

#### Step 1: Find a cable

To begin with, find a male USB-A to anything cable.

If you have purchased the suggested components listed in [](#get-db-hw), you can use the longer USB cable contained inside the battery package ([](#figure:battery-pack-usb-cables)), which will be used as an example in these instructions.

<div figure-id="fig:battery-pack-usb-cables" figure-caption="The two USB cables in the suggested battery pack.">
     <img src="battery-pack-usb-cables.jpg" style='width: 15em'/>
</div>

Put the shorter cable back in the box, and open the longer cable ([](#figure:long-usb-cable))

<div figure-id="fig:long-usb-cable" figure-caption="Take the longer cable, and put the shorter on back in the box.">
     <img src="long-usb-cable.jpg" style='width: 15em'/>
</div>

#### Step 2: Cut the cable

<div class="check" markdown="1">
Make sure the USB cable is _unplugged_ from any power source before proceeding.
</div>

 Take the scissors and cut it ([](#figure:cut-the-cable)) at the desired length from the USB-A port.

<div figure-id="fig:cut-the-cable" figure-caption="Cut the USB cable using the scissors.">
     <img src="cut-the-cable.jpg" style='width: 15em'/>
</div>

The cut will look like in [](#figure:the-cable-cut).

<div figure-id="fig:the-cable-cut" figure-caption="A cut USB cable.">
     <img src="the-cable-cut.jpg" style='width: 15em'/>
</div>

#### Step 3: Strip the cable

Paying attention not to get hurt, strip the external white plastic. A way to do so without damaging the wires is shown in [](#figure:stripping-the-outside).

<div figure-id="fig:stripping-the-outside" figure-caption="Stripping the external layer of the USB cable.">
     <img src="stripping-the-outside.jpg" style='width: 15em'/>
</div>

After removing the external plastic, you will see four wires: black, green, white and red ([](#figure:under-the-hood)).

<div figure-id="fig:under-the-hood" figure-caption="Under the hood of a USB-A cable.">
     <img src="under-the-hood.jpg" style='width: 15em'/>
</div>

Once the bottom part of the external cable is removed, you will have isolated the four wires ([](#figure:stripped-the-outside)).

<div figure-id="fig:stripped-the-outside" figure-caption="The four wires inside a USB-A cable.">
     <img src="stripped-the-outside.jpg" style='width: 15em'/>
</div>

#### Step 4: Strip the wires {#strip-the-power-wires}

<div class="check" markdown="1">
Make sure the USB cable is _unplugged_ from any power source before proceeding.
</div>

Once you have isolated the wires, strip them, and use the scissors to cut off the data wires (green and white, central positions) ([](#figure:strip-power-wires)).

<div figure-id="fig:strip-power-wires" figure-caption="Strip the power wires and cut the data wires.">
     <img src="strip-power-wires.jpg" style='width: 15em'/>
</div>

If you are not using the suggested cable, or want to verify which are the data and power wires, continue reading.

#### Step 5: Find the power wires

If you are using the USB-A cable from the suggested battery pack, black and red are the power wires and green and white are instead for data.

If you are using a different USB cable, or are curious to verify that black and red actually are the power cables, take a multimeter and continue reading.

Plug the USB port inside a power source, e.g., the Duckiebot's battery. You can use some scotch tape to keep the cable from moving while probing the different pairs of wires with a multimeter. The voltage across the pair of power cables will be roughly twice the voltage between a power and data cable. The pair of data cables will have no voltage differential across them. If you are using the suggested Duckiebot battery as power source, you will measure around 5V across the power cables ([](#figure:testing-voltage-usb-cable-power-wires)).

<div figure-id="fig:testing-voltage-usb-cable-power-wires" figure-caption="Finding which two wires are for power.">
     <img src="testing-voltage-usb-cable-power-wires.jpg" style='width: 15em'/>
</div>

#### Step 6: Test correct operation

You are now ready to secure the power wires to the DC motor HAT power pins. To do so though, you need to have soldered the boards first. If you have not done so yet, read [](#assembling-duckiebot-db17-soldering).

If you have soldered the boards already, you may test correct functionality of the newly crafted cable. Connect the battery with the DC motor HAT by making sure you plug the black wire in the pin labeled with a minus: `-` and the red wire to the plus: `+` ([](#figure:final-result-power-c0)).

<div figure-id="fig:final-result-power-c0" figure-caption="Connect the power wires to the DC motor HAT">
     <img src="final-result-power-c0.jpg" style='width: 15em'/>
</div>

# Bumper Assembly {#bumper-assembly status=beta}

<div class='requirements' markdown="1">

Requires: Duckiebot `DB17-lc` parts.

Requires: Having the Duckiebot with configuration `DB17-wjd` assembled. The assembly process is explained in [](#assembling-duckiebot-db17jwd).

Requires: Time: about 15 minutes.

Results: A Duckiebot with Bumpers (configuration DB17-l2)

</div>

## Locate all required parts

The following should be included in your parts envelope (See image below for these components):

* 1x front bumper (Camera side)
* 1x rear bumper (the side of Caster/Omnidirectional wheel)
* 2x rear bumper brace (the side of Caster/Omnidirectional wheel)
* 8x M2.5x10 nylon or metal screws
* 8x M2.5 nuts

The following is not included in your parts envelope but will be needed for assembly:

* Small screwdriver

<div figure-id="fig:components-package" figure-caption="Components in Duckiebot package.">
    <img src="Bumpers.jpg" style='width:30em; height:auto'/>
</div>

<div figure-id="fig:screws_for_bumper" figure-caption= "screws for fasten the bumpers">
    <img src="screws_bumpers.jpg" style='width:30em; height:auto'/>
</div>

### Reminder: Use care when assembling! {#assembly-with-care-db17}

 Use care when assembling, make sure to be gentle! Tighten screws just enough so that the parts will remain stationary. When inserting LEDs, gently work them into their holders. While the acrylic is relatively tough, it can be fractured with modest force. We don’t have many replacements (at this moment) so we may not be able to replace a broken part.

### Remove protective paper

Peel protective layer off of all parts on all sides.

<div figure-id="fig:remove_schild" figure-caption= "Bumpers with its protective layer">
    <img src="remove_schild.jpg" style='width:30em; height:auto'/>
</div>


### Assembly Rear Spacers

#### Disassembly the spacers between the both chassis

The backside of duckiebot before assemblying the bumpers looks as [](#fig:before_bumpers):
<div figure-id="fig:before_bumpers" figure-caption= "The configuration before assemblying the bumpers">
    <img src="before_bumpers.jpg" style='width:30em; height:auto'/>
</div>

Now remove the spacers and the (short)metall screws from the standoffs (configuration 'DB17-wjd') and replace it with 4 M3x10 nylon screws for connecting the chassis and the bumper spacers.

<div figure-id="fig:screw_for_upgrade" figure-caption= "The spacers configuration for the bumpers, Left: old configuration, Right: new configuration">
    <img src="screwForUpgrade.jpg" style='width:30em; height:auto'/>
</div>

<!--Disassemble the rear spacers. KEEP ALL HARDWARE (you will be using the 2 short screws that were connected to the bottom of the spacer later). Reassemble in the configuration shown in the images below (note that the rear bumper braces now act as spacers to preserve the height offset). Reinstall the longer M2.5 screws that were originally connected at the top, and replace the bottom screws with 2 of your M3x10 screws included in the envelope. You may need the pliers to grasp the hex spacers when tightening. Note [Reminder: Use care when assembling!](#assembly-with-care-db17) -->

M3x10 screws attaching bottom rear brace:

<div figure-id="fig:bottom_rear_brace" figure-caption= "Attach the spacers with M3*10 nylone screws">
    <img src="bottom_rear_brace.jpg" style='width:30em; height:auto'/>
</div>

Back View, fully assembled:

<div figure-id="fig:back_view_fully_assembled" figure-caption="Back view of rear spacers">
    <img src="back_view_fully_assembled.jpg" style='width:30em; height:auto'/>
</div>

Note: For the ETH students, the M3*10 nylon screws for attaching the rear spacers with chassis were already distributed during the duckiebot ceremony and not included in second distribution. Please reuse them!

#### Mount Rear Bumper

Carefully guide rear bumper on to rear bumper brace tabs. Ensure that the hole for charging aligns with the charging port on your battery.

<div figure-id="fig:rear_bumpers" figure-caption="Front view of rear bumper">
    <img src="rear_bumpers.jpg" style='width:30em; height:auto'/>
</div>

Locate 4 M2.5 nylon/metall nuts and 4 M2.5*10 nylon screws. Place a nut in the wide part of the t-slot and thread a screw into the nut as shown in the following pictures. Note [Use care when assembling!](#assembly-with-care-db17) If you are having trouble with the nuts falling out, take a small piece of transparent tape and place it over both sides of the t-slot with the nut inside. It won’t look as nice but it will be much easier to assemble.

Better: Test the screws and the nuts once by screwing them together before you use it for the bumpers. It make the fllowing assembly process much easier.

Note: For ETH 2017, the screws and nuts using for this step are M2.5*10 nylon screws (white) and M2.5 nylon or metall nuts from the envelope.

<div figure-id="fig:bumper_screw1" figure-caption = "Hold the nuts with the fingers">
    <img src="bumper_screw1.jpg" style='width:30em; height:auto'/>
</div>
<div figure-id="fig:bumper_screw2" figure-caption = "Screw the screws into nuts while holding the nuts with the fingers">
    <img src="bumper_screw2.jpg" style='width:30em; height:auto'/>
</div>


The completed rear bumper should look like this:

<div figure-id="fig:bumper_figure_12" figure-caption = "Completed rear bumpers">
    <img src="image_12-1.jpg" style='width:30em; height:auto'/>
</div>

Congrats! your rear bumper assembly is complete!

#### Mount Front Bumper

The front side of Duckiebot in `DB17-wjd` should look like in [](#fig:frontside_before_bumper).

Take 2x M2.5*10 nylon screws and 2x M2.5 nylon nuts and install them as shown in the following pictures. The first picture shows the correct holes to mount these screws (The correct position is the widest pair of 3mm holes beside the camera). The nuts should tightened on by a few threads (these are the two nuts that are not yet tightened at the top of the second picture):

<div figure-id="fig:frontside_before_bumper" figure-caption="Front side of configuration DB17-wjd, without bumpers">
    <img src="frontside_before_bumper.jpg" style='width:30em; height:auto'/>
</div>

Better: Before tighten the front bumper, you should organize the wires of LEDs going through the right holes of chassis. The center LED should be bent at a right angle in the direction that the wire is fed through the body.

Take the front bumper and carefully press the LEDs into the bumper holders. Take care that the wires are routed behind the front bumper. Also note that the front center LED wire should not be crushed between the bumper and the right spacer (you will likely fracture the bumper if you try to force it). The center LED should be bent at a right angle in the direction that the wire is fed through the body (see [](#fig:front_bumper_LED)).

<div figure-id="fig:front_bumper_LED" figure-caption="Insert the LEDs before tighten the front bumper.">
    <img src="front_bumper_LED.jpg" style='width:30em; height:auto'/>
</div>

Position the bumper so that the nuts align with the t-slots. You may need to loosen or tigthen the screws to align the nuts. You may also need to angle the front bumper when inserting to get it
past the camera screws.

<div figure-id="fig:bumper_figure_17" figure-caption = "Completed front bumpers">
    <img src="front_bumper_LED2.jpg" style='width:30em; height:auto'/>
</div>

Gently tighten the nuts. The front bumper should now stay in position.

## Assembly instructions (`DB17-l`) {#assembling-duckiebot-db17l status=beta}

<div class='requirements' markdown="1">

Requires: Duckiebot `DB17-lc` parts. The acquisition process is explained in [](#get-db17-hw).

Requires: Soldering `DB17-lc` parts. The soldering process is explained in [](#assembling-duckiebot-db17-soldering).

Requires: Having assembled the Duckiebot in configuration `DB17` (or any `DB17-wjd`). The assembly process is explained in [](#assembling-duckiebot-db17jwd).

Requires: Time: about 30 minutes.

Results: An assembled Duckiebot in configuration `DB17-wjdlc`.

</div>

### Assembly the Servo/PWM hat (`DB17-l1`)

Recommend: If you have bumpers, it is recommend to have them assembled before the PWM hat. The assembly process is explained in [](#bumper-assembly).

#### Locate the components for Servo/PWM hat

* Soldered PWM hat (1x)
* Nylon Standoffs (M3.5 12mm F-F) (4x)
* Power cable: short angled male USB-A to 5.5/2.1mm DC power jack cable
* Male-Male Jumper Wires (1x)
* Screwdriver

<div figure-id="fig:component_PWM" figure-caption="Components-List for PWM HAT">
     <img src="component_PWM.jpg" style='width: 30em'/>
</div>

#### Remove the hand-made USB power cable from DC Motor HAT

From now on, the DC Motor Hat will be powered by the PWM HAT via male -male jumper wire. Before that, the previous hand-made USB power cable needed to be removed. Insert the male-male jumper wire into `+` power terminal on the DC motor HAT (DC-end).

<div figure-id="fig:positive_terminal_DC" figure-caption="Insert the male-male wire into `+` terminal block on the DC motor HAT">
     <img src="plus_terminal_DC.jpg" style='width: 30em'/>
</div>

#### Stack the PWM HAT above the DC motor HAT

Put a soldered Servo/PWM HAT board (in your Duckiebox) with 4 standoffs on the top of Stepper Motor HAT.

Insert the other end of male-male jumper wire into "**+5**"V power terminal on the PWM HAT (PWM-end). It leads the power to DC motor HAT.

<div figure-id="fig:pos_terminal_PWM" figure-caption="Insert the PWM-end into +5V terminal on PWM HAT ">
     <img src="pos_terminal_PWM.jpg" style='width: 30em'/>
</div>


#### Power Supply for PWM HAT

To power the PWM/Servo HAT from the battery, plugin a short (30cm) angled male USB-A to 5.5/2.1mm DC power jack cable into PWM HAT. The other end of the power cable will plugin to the battery when it is in use.  

<div figure-id="fig:angled_power_cable" figure-caption="Plugin the short angled male DC power cable">
     <img src="angled_power_cable.jpg" style='width: 30em'/>
</div>

### Assembling the Bumper Set (`DB17-l2`)

For instructions on how to assemble your bumpers set, refer to: [](#bumper-assembly).

### Assembling the LED HAT and LEDs (`DB17-l3`)

For instructions on how to assemble the LED HAT and related LEDs, refer to: [](#leds-setup-db17).

TODO: finish above, estimate assembly time, add bumper assembly instructions, add LED positioning and wiring instructions

## Assembling the `DB17-lc` {#leds-setup-db17 status=beta}
