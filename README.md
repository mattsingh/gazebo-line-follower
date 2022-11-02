## Launch Gazebo Environment

The `turtlebot3_world.launch` map is temporary until we find a way to make a map with yellow lines.

```
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

> **_NOTE:_**  The 'burger' model doesn't have a camera whereas the 'waffle_pi' does. Add `export TURTLEBOT3_MODEL=waffle_pi` to .bashrc to avoid having to type this often.