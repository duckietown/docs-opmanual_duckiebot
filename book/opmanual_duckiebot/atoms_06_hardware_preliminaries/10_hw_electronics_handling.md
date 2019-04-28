# Handling circuits and batteries {#db-opmanual-preliminaries-electronics status=ready}

<div class='requirements' markdown="1">

Requires: Nothing

Results: Preliminary knowledge on circuits and power source properties useful in Duckietown

</div>

Duckiebots support several power bank models, although not any power will work. Here, we list properties of the supported models.

## The Duckie-power-bank

The Duckie-power-bank (or simply duckiebattery) is the standard power source for Duckiebots in `DB18` configuration. Duckiebatteries are easily recognizable.

<div figure-id="fig:duckiebattery-front" figure-caption="The Duckie Power Bank">
     <img src="duckiebattery-front.png" style='width: 37em' />
</div>

### Overview

The duckiebattery is equipped with 2 USB type A outputs (port A and B) and 1 Micro USB connector for charging.

<div figure-id="fig:duckiebattery-ports" figure-caption="The Duckie Power Bank ports">
     <img src="duckiebattery-ports.png" style='width: 37em' />
</div>

It also has 4 LEDs representing the state of charge. Push the button on the side of the battery pack to turn on the LEDs. The LEDs indicate the residual charge according to:

<div markdown="1">
 <col2 id='duckiebattery-charge' figure-id="tab:duckiebattery-charge" figure-caption="Duckiebattery LED charge indicators">
    <s>D1</s>                         <s>3-25%</s>
    <s>D2</s>                         <s>25-50%</s>
    <s>D3</s>                         <s>50-75%</s>
    <s>D4</s>                         <s>75-100%</s>
 </col2>
</div>

If D1 is flashing (0.5Hz) while not being charged, the battery pack is at a critical low charge ( less than 3%).

### Charging

The battery pack is charged via the Micro USB port with a 5V supply. While charging, one of the LEDs will be flashing (0.5Hz) indicating where in the charge cycle it is.

Note: When the battery pack is connected to the charger, the output voltage of port A and B will turn off for around 280ms:

<div figure-id="fig:duckiebattery-voltage-profile" figure-caption="The Duckie Power Bank ports">
     <img src="duckiebattery-voltage-profile.png" style='width: 37em' />
</div>

This is an unwanted effect which will cause the Raspberry Pi, if on, to reboot.

Note: likewise, when disconnecting the charger, the outputs will turn OFF for 20ms causing the Raspberry Pi to reboot as well.

Furthermore, while the duckiebattery supports pass-through (both outputs A and B will function while the duckiebattery is being charged), during charge the output voltage of port A and B will drop around 300mV which might cause an under voltage warring of the Raspberry Pi. This will put the Raspberry Pi in a throttling mode limiting its performance.

### Discharging

The output ports A and B have an unloaded output voltage around 5.1V. To turn the outputs on simply attach a load on port A/B (e.g., plug in to the Raspberry PI and duckieboard) or push the button.

The output ports will automatically turn off if less than 100mA is being drawn.

To turn the outputs back on simply push the button or reconnect the USB connector.  

The combined output current is limited to 2.8A.

The battery capacity is 7.4Ah at 5V with an efficiency as follows:

| Load  &nbsp; &nbsp; |      Efficiency &nbsp; &nbsp;     |  Autonomy &nbsp; &nbsp; |
|----------|:-------------:|------:|
| 1 A |  91% | 6h 44m |
| 1.5 A |    88%   |   4h 33m |
| 2 A | 85% |    3h 9m |
| 2.5 A | 79% |    2h 21m |
