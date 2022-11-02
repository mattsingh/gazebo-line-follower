## Launch Gazebo Environment

```
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_gazebo turtlebot3_autorace.launch
```

> **_NOTE:_**  The 'burger' model doesn't have a camera whereas the 'waffle_pi' does. Add `export TURTLEBOT3_MODEL=waffle_pi` to .bashrc to avoid having to type this often.