# Duckietown Appearance Specification {#duckietown-specs status=beta}

This document describes the Duckietown specification. These are a set of rules for which a functional system has been verified.

Any Duckietown not adhering to the rules described here cannot call itself a "Duckietown", since it is not one.

Additionally, any Duckietown not adhering to these rules may cause the Duckiebots to fail in unexpected ways.

## Version history

Note here the changes to the specification, so that we are able to keep in sync
the different Duckietowns.

* Version 1.0 - used for MIT 2.166

* Version 2.0 - user for Fall 2017

## Overview

Duckietown is built with two layers:

1. The first is the *floor layer*. The floor is built of interconnected exercise mats with tape on them.
2. The second layer is the *signals layer* and contains all the signs and other objects that sit on top of the mats.

Note: the visual appearance of the area where the Duckietown is created is variable. If you discover that this appearance is causing negative performance, a "wall" of blank tiles constructed vertically can be used to reduce visual clutter.


## Layer 1 - The Tile Layer

Each tile is a 2 ft x 2 ft square and is able to interlock with the others.

There are six types of tiles, as shown in [](#fig:tiles).
Currently, the left turn and right turn tiles are symmetric: one is the 90 degree rotation of the other.


<div figure-id="fig:tiles" figure-class="flow-subfigures" figure-caption="The principal tile types in Duckietown">
    <div figure-id="subfig:straight" figure-caption="DT17_tile_straight">
        <img src="DT17_tile_straight-texture.png" style='width: 20ex'/>
    </div>
    <div figure-id="subfig:DT17_tile_curve_left" figure-caption="DT17_tile_curve_left">
        <img src="DT17_tile_curve_left-texture.png" style='width: 20ex'/>
    </div>
    <div figure-id="subfig:DT17_tile_curve_right" figure-caption="DT17_tile_curve_right">
        <img src="DT17_tile_curve_right-texture.png" style='width: 20ex'/>
    </div>
    <div figure-id="subfig:DT17_tile_three_way_center" figure-caption="DT17_tile_three_way_center">
        <img src="DT17_tile_three_way_center-texture.png" style='width: 20ex'/>
    </div>
    <div figure-id="subfig:DT17_tile_four_way_center" figure-caption="DT17_tile_four_way_center">
        <img src="DT17_tile_four_way_center-texture.png" style='width: 20ex'/>
    </div>
    <div figure-id="subfig:DT17_tile_empty" figure-caption="DT17_tile_empty">
        <img src="DT17_tile_empty-texture.png" style='width: 20ex'/>
    </div>
</div>


<div figure-id="fig:DT17_map_loop3" figure-caption="A 3 by 3 loop (DT17_map_loop3)">
    <img src="DT17_map_loop3-texture.png" style='width: 8cm'/>
</div>


<div figure-id="fig:DT17_usage_four_way" figure-caption="Four way intersection usage">
    <img src="DT17_usage_four_way-texture.png" style='width: 8cm'/>
</div>
<div figure-id="fig:DT17_usage_three_way" figure-caption="Three way intersection usage">
    <img src="DT17_usage_three_way-texture.png" style='width: 8cm'/>
</div>


### Tapes

There are 3 colors of tapes: white, yellow, and red.

#### White tape 

\begin{proposition}\label{prop:white_tape}
A Duckiebot never collides with Duckietown if it never crosses or touches a white tape strip.
\end{proposition}

Here are some facts about the white tapes:

* White tapes must be solid (not dashed)

* The width of the white tape is 2 inches (5.08 cm).

* The white tape is always placed on the right hand side of a lane. We assume that the Duckiebots drive on the right hand side of the road.

Comment: this should be part of the "traffic rules" sections.

* For curved roads, the white lane marker is formed by five pieces of white tape, while the inner corner is formed by three pieces, placed according to the specifications in the image below, where the edge pieces are matched to adjacent straight or curved tiles ([](#fig:curved)).

<div figure-id="fig:curved" figure-caption="The specification for a curved road tile">
  <img src="curved_road.png" style='width: 30em; height:auto'/>
</div>


#### Yellow tape

On a two-way road, the yellow tape should be dashed. Each piece should have a length of approximately **2 in** with a **1 in** gap separating each piece.

Yellow tapes on curves: see curved road image in white tape section, pieces at tile edges should be in center of lane, piece at the middle of the curve should be approximately 20.5 cm from middle of inner center white piece of tape, with approximated circular arc in between.

#### Red tape

Red tapes MAY **only** appear on **intersection** tiles.

The red tape must be the full width of the duck tape roll and should cross the entire lane perpendicular to the lane.

The placement of red tape should always be **under** yellow and white tape.

A Duckiebot navigates Duckietown by a sequence of:

* Navigating one or more straight ties until a red tape appears,
* Wait for the coordination signal,
* Execute an intersection traversal,
* Relocalize in a StraightTile.

The guarantee is:

\begin{proposition}
If the Duckiebot stops before or ON the red strip, no collisions are possible.
\end{proposition}

### Topological Constraints During Map Construction

Here are some topological rule constraints that must be met:

1. An intersection can NOT be adjacent to a curved road tile or another intersection tile.

2. Any two adjacent non-empty tiles must have a feasible path from one to the other **of length two**: if they are adjacent, they must be connected.

Some examples of **non-conforming** topologies are shown in [](#fig:violates).

<div figure-id="fig:violates" figure-class="flow-subfigures" figure-caption="Some non-conforming Duckietown map topologies">
    <div figure-id="subfig:violates1" figure-caption="Topology violates rule 2 since the bottom two curved tiles are adjacent but not connected">
        <img src="violates1.pdf" style='width: 20ex;height:auto'/>
    </div>
    <div figure-id="subfig:violates2" figure-caption="Topology violates rule 1 since curved tiles are adjacent to intersection tiles ">
        <img src="violates2.pdf" style='width: 20ex;height:auto'/>
    </div>
    <div figure-id="subfig:violates3" figure-caption="Topology violates rule 2 since left-most tiles are adjacent but not connected">
        <img src="violates3.pdf" style='width: 20ex;height:auto'/>
    </div>
</div>


### Parking Lots {#parking}

Note: An experimental new development.

A parking lot is a place for Duckiebots to go when they are tired and need a rest.

A parking lot introduces three additional tile types:

1. **Parking lot entry tile**: This is similar to a straight  tile except with a red stop in the middle. The parking lot sign ([](#fig:parking)) will be visible from this stop line.
2. **Parking spot tiles**:
3. **Parking spot access tiles**:

TODO: the tape on the spot and spot access tiles is currently not yet specified.

The following are the rules for a conforming parking lot:

1. One "parking spot" has size one tile.
2. From each parking spot, there is a path to go to the parking lot entry tile that does not intersect any other parking spot. (i.e. when a Duckiebot is parked, nobody will disturb it).
3. From any position in any parking spot, a Duckiebot can see at least two orthogonal lines or an sign with an April tag.

TODO: this point needs further specification


### Launch Tiles {#launch-tiles}

Note: Experimental

A "launch tile" is used to introduce a new Duckiebot into Duckietown in a controllable way. The launch file should be placed adjacent to a turn tile so that a Duckiebot may "merge" into Duckietown once the initialization procedure is complete.

TODO: Specification for tape on the launch tile

A "yield" sign should be visible from the launch tile.

## Layer 2 - Signage and Lights

**IMPORTANT:** All signage should sit with base on the floor and stem coming through the connection between the tiles. Generally, it is advisable to adhere the sign to the floor with double-sided tape. **Under no circumstances should the white (any other tape) be obscured.**

## Traffic Signs {#traffic-signs status=beta}

Requires: To print and assemble the signs refer to [](#signage).

### Specs

Center of signs are 13 cm height with apriltags of 6.5 cm sq. and a white border pasted below them.

### Type

The allowable traffic signs are as in [](#fig:traffic-signs).


<div figure-id="fig:traffic-signs" figure-class="flow-subfigures" figure-caption="Duckietown Traffic Signs">
  <div figure-id="subfig:stop" figure-caption="stop">
    <img src="stop.png" style='width:8em;height:auto'/>
  </div>
  <div figure-id="subfig:yield" figure-caption="yield">
    <img src="yield.png" style='width:8em;height:auto'/>
  </div>
  <div figure-id="subfig:no-right" figure-caption="no-right-turn">
    <img src="no-right.png" style='width:8em;height:auto'/>
  </div>
  <div figure-id="subfig:no-left" figure-caption="no-left-turn">
    <img src="no-left.png" style='width:8em;height:auto'/>
  </div>
  <div figure-id="subfig:no-enter" figure-caption="do-not-enter">
    <img src="no-enter.png" style='width:8em;height:auto'/>
  </div>
  <div figure-id="subfig:one-way-right" figure-caption="oneway-right">
    <img src="one-way-right.png" style='width:8em;height:auto'/>
  </div>
  <div figure-id="subfig:one-way-left" figure-caption="oneway-left">
    <img src="one-way-left.png" style='width:8em;height:auto'/>
  </div>
  <div figure-id="subfig:4-way-intersect" figure-caption="4-way-intersect">
    <img src="4-way.png" style='width:8em;height:auto'/>
  </div>
  <div figure-id="subfig:3-way-right" figure-caption="right-T-intersect">
    <img src="3-way-right.png" style='width:8em;height:auto'/>
  </div>
  <div figure-id="subfig:3-way-left" figure-caption="left-T-intersect">
    <img src="3-way-left.png" style='width:8em;height:auto'/>
  </div>
  <div figure-id="subfig:t-intersection" figure-caption="T-intersection">
    <img src="t-intersection.png" style='width:8em;height:auto'/>
  </div>
  <div figure-id="subfig:crossing" figure-caption="pedestrian">
    <img src="crossing.png" style='width:8em;height:auto'/>
  </div>
  <div figure-id="subfig:traffic-light" figure-caption="t-light-ahead">
    <img src="traffic-light.png" style='width:8em;height:auto'/>
  </div>
  <div figure-id="subfig:duckie-crossing" figure-caption="duck-crossing">
    <img src="duckie-crossing.png" style='width:8em;height:auto'/>
  </div>
  <div figure-id="subfig:parking" figure-caption="parking">
    <img src="parking.png" style='width:8em;height:auto'/>
  </div>
</div>



### Placement {#traffic-signs-placement status=beta}

Signs may appear on the opposite side and at the corner of the adjacent tile from which they are viewed. In the absence of any signs, it is assumed that all network flows are allowed so a sign MUST be placed and visible whenever this is not the case.

Signs must only be placed on empty tiles, or next to one of the other tile types if on the border of a map. The sign placements for four different cases are shown in [](#fig:sign-placement). At intersections, from each stop line 2 signs should be clearly visible: 1) the intersection type (traffic light or stop sign) and 2) the intersection topology.

At present, 4-way intersections must be equipped with traffic lights for safe navigation.

<div figure-id="fig:sign-placement" figure-class="flow-subfigures" figure-caption="Placement of Traffic Signs">
  <div figure-id="subfig:4-way-signs" figure-caption="4-way intersection">
    <img src="4-way-signs.pdf" style='width:15em;height:auto'/>
  </div>
  <div figure-id="subfig:3-way-signs" figure-caption="3-way intersection">
    <img src="3-way-signs.pdf" style='width:15em;height:auto'/>
  </div>
  <div figure-id="subfig:2-way-signs-straight" figure-caption="straight road">
    <img src="2-way-signs-straight.pdf" style='width:15em;height:auto'/>
  </div>
  <div figure-id="subfig:2-way-signs-turn" figure-caption="curved road">
    <img src="2-way-signs-turn.pdf" style='width:15em;height:auto'/>
  </div>
</div>

On straight and curved roads, additional signs can be added as desired. Their placement is indicated in [](#subfig:2-way-signs-straight) and [](#subfig:2-way-signs-turn). The signs should be placed at the border between two tiles and should face towards oncoming traffic as indicated.

In these figures the arrow is the direction of the sign.


## Street Name Signs {#street-name-signs status=beta}

### Specs


* Font: arial.

* Color: Perhaps we could start with real-world settings: white as foreground and green as background.

* Border: currently no additional borders

* The rounded corners are modified into 90 degrees.

* Height: sign board height is 1.5 in. (**2.1 in**),

* Width: Currently 4.5 in for id 500-511. (**6.1 in +1.1 in "ST" or 5.5 in + 1.7 in “AVE”**)

* Alphabet =  English upper case. Different writing systems may need different algorithms.

* Text direction: Horizontal for alphabetical languages


### Placement

* **Similar to traffic light**: The street name should sit on a pole that is based at the corner of the tile outside of the allowable driving region. The bottom of the street name should be at a height of 7in, and allow a duckiebot to pass through. The street names should be visible from both sides of the road.

Every segment of road must have at least one road name sign.

Every turn tile should have a road name sign.

The placement of the road name signs is as indicated in [](#fig:name-placement).

<div figure-id="fig:name-placement" figure-class="flow-subfigures" figure-caption="Placement of Road Name Signs">
  <div figure-id="subfig:name-signs-turn" figure-caption="Turn">
    <img src="name-signs-turn.pdf" style='width:15em;height:auto'/>
  </div>
  <div figure-id="subfig:name-signs-straight" figure-caption="Straight">
    <img src="name-signs-straight.pdf" style='width:15em;height:auto'/>
  </div>
</div>



Street name signs should never be perpendicular to the road - they are too big and obtrusive.



## Traffic Lights

Requires: The assembly procedure for building the a traffic light is found in [](#traffic-light-assembly)

### Specs

TOWRITE: towrite

### Placement

The lights must be at a height of exactly 20 cm  above the center of the intersection tile.

The Raspberry Pi should sit on a pole that is based at the corner of the tile outside of the allowable driving region.
