## Setup Environement
Add the two maps with built in lines by copying the `*.world` files into turtlebot's worlds folder located at:
`~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds`
Add the launch configuration for the two maps by copying the `*_world.launch` files into turtlebot's launch folder located at:
`~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch`

Source these changes by executing the following in the terminal
```
. ~/catkin_ws/devel/setup.bash
```

## Launch Gazebo Environment

The `turtlebot3_world.launch` map is temporary until we find a way to make a map with yellow lines.

```
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_gazebo lfm1.launch
```

> **_NOTE:_**  The 'burger' model doesn't have a camera whereas the 'waffle_pi' does. Add `export TURTLEBOT3_MODEL=waffle_pi` to .bashrc to avoid having to type this often.