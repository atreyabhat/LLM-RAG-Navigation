# LLM-Navigation

#### Install Nav2
```bash
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
```

```bash
export TURTLEBOT3_MODEL=waffle
```

#### Setting up the Environment for TurtleBot3

1) Place all the "aws_robomaker_models" in /usr/share/gazebo-11/models
2) Place the launch files in the /opt/ros/humble/share/turtlebot3_gazebo/launch
3) Place the world files in the /opt/ros/humble/share/turtlebot3_gazebo/worlds

#### Open the Simulation on Gazebo
```bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

```bash
ros2 launch turtlebot3_gazebo turtlebot3_bookstore.launch.py
```

