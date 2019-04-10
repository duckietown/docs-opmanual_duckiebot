# Intersection troubleshooting {#trouble-unicorn_intersection status=draft}

If your Duckiebot does not yield satisfactory results in intersection navigation, you might want to tune the related parameters.
Basically the `unicorn_intersection_node` (long story for the name) is a mixture of open loop commands and a re-use of the lane filter.
During the intersection, namely when the Duckiebot is in the `FSM` state `INTERSECTION_CONTROL`, the color perception of lines is changed. As a simple example if the goal is to go straight, the red lines will be perceived as white, so that it will be possible to follow the right white line. On top of this there are a few open loop commands that are used to help the Duckiebot face the correct direction. These parameters are stored in

    `Software/catkin_ws/src/00-infrastructure/duckietown/config/baseline/unicorn_intersection/unicorn_intersection_node/default.yaml`

You can change them online (while the demo is running) by using:

     duckiebot-container $ rosparam set ![your_parameter] ![your value]

You can see all the parameters by running:

    duckiebot-container $ rosparam list

And check the value of a specific one using:

    duckiebot-container $ rosparam get ![param name]

The ones you might want to modify are the feed-forward parts, stored in `ff_left`, `ff_right` and `ff_straight`. These parameters modify the output `$\omega$` for the time given in `time_left_turn`, `time_straight_turn` and `time_right_turn`, which you might want to change aswell.
