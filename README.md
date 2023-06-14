# AR steering
This repository is a ROS package that contains AR steering demo showing the capabilities of controling the Robotont platform with AR Tag.

## Prerequisites installed on the Robotont on-board computer
 * [ar_track_alvar](http://wiki.ros.org/ar_track_alvar)

## Launching the demo
The AR Tag id can be specified as an argument.

```bash
roslaunch ar_steering ar_steering.launch [marker_id:=10]
```