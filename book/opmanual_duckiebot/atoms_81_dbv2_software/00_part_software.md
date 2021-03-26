# Duckiebot V2 Software Overview {#part:dbv2-software status=beta}

The DBV2 software is kept separate from the main Daffy software.
However, DBV2 still uses most of the same software as DB18. Therefore, the DBV2 software is kept in separate
Docker containers which build on top of the Daffy containers. Each one replaces a small part of the Daffy
software, and adds new functionality specific to DBV2:

 - [`dt-duckiebot-interface-dbv2`](https://github.com/duckietown/dt-duckiebot-interface-dbv2): 
   Contains the drivers for all of the sensors on DBV2, as well as a new wheels driver.
 - [`dt-car-interface-dbv2`](https://github.com/duckietown/dt-car-interface-dbv2): 
   Contains a new kinematics node for DBV2
 - [`dt-core-dbv2`](https://github.com/duckietown/dt-core-dbv2): 
   Contains all DBV2-specific demos, such as lane following with line following sensors

In the following sections, each of these containers and each package and node within is documented.
