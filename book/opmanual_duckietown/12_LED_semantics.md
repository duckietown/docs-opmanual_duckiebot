# Semantics of LEDS {#LED-semantics status=draft}

Assigned: ???

headlights: white, constant

Assumption:

- **20 fps** to do LED detection

- 1s to decide

- 3 frequencies to detect


tail lights: red, **6 hz square wave**

traffic light "GO" = green, 1 hz** square wave**

traffic light "STOP" = red, 1.5 Hz** square wave**

duckie light on top, state 0 = off

duckie light on top, state 1 = blue, **3 Hz, square wave**

duckie light on top, state 2 = ?, **2.5 Hz square wave**

duckie light on top, state 3 = ?,** ****2 Hz square wave**
