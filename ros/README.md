# Map-less Navigation with Obstacle avoidance

This directory contains the implementation of the DDQN on the Turtlebot in ros. 

## Delpendencies 

* [matio](https://github.com/tbeu/matio)
* [MiniDNN](https://github.com/yixuan/MiniDNN)
* [turtlebot packages](http://wiki.ros.org/turtlebot) (ros)

## Run Project

1. launch project
    ```
    roslaunch turtlebot_dqn turtlebot_arl.launch
    ```

2. load config file in folder [Rviz](ros/rviz)

3. run keyboard_node in another terminal
    ```
    rosrun user_interface keyboard_node
    ```


## Contributions

### virtual_walls_node

In order to ensure that the robot remains in a predifined area, we introduced the [virtual_walls_node](ros/virtual_walls_node) which projects the laser range data on a virtual rectangular region. This brings the benefits, that we reduce the noise of laser range data and operate the Turtlebot in a controlled environment. Furthermore, this increases the demands on the DQN-controlled agent.

This node subscribes to `/lidarscan` and publishes the topic `/virtualscan`.


###  user_interface_node

For a convenient user interaction with the Turtlebot we programmed the [user_interface_node](ros/user_interface_node) which is implemented as state machine. By pressing *ENTER* the node switches between the states `RUN` and `INTERRUPT`. If *ESC* is pressed the Turtlebot resets its odometry. (This is required because in our implementation we rely on `/odom` which drifts after several cycles). 

### turtlebot_dqn_node

This node implements the DDQN learning and acts as inteface to the Turtlebot. It preprocesses the laser range data and convert them into the desired 18 rays. Moreover, the distance to the target *d* and the heading difference *θ* with respect to the target location were calculated. ​

The DDQN parameter configuration is provided [here](../snippets/dqn).

The input topics are `/virtualscan` and `/move_base_simple/goal` (navigation goal in Rviz).
Additionally it requires the tf tree with the transformations between `/odom` and `/base_link`. 

The output topic is `/cmd_vel_mux/input/teleop`.
