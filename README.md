autocp
======

Automatic Camera Placement for pr2_interactive_manipulation

In development.

This rviz plugin automatically moves the camera in pr2_interactive_manipulation to get a good view, which will hopefully eliminate the need to move the camera around manually.

The camera tracks the average point between the grippers and the head target point. If the user is using an interactive marker, the camera will randomly search around its current position every half second or so, to see if it can find a better position. If it does, it moves there.

A "better" position is currently defined as:

* Orthogonal to the current control being manipulated.
* Not occluded by other objects.
* Not too far away from the current position.

Those factors are weighted to produce the final score.

To build: `rosmake autocp`

To run: launch pr2_interactive_manipulation and add "AutoCPDisplay" as a display.
