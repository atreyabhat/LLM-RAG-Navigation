# LLM-Navigation

#### Install Nav2
```bash
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
```
#### Install Turtlebot3
```bash
sudo apt install ros-humble-turtlebot3*
```
Add key environment variable
```bash
gedit ~/.bashrc
```
Paste the below line in .bashrc
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

#### Moving the Turtlebot3
```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

#### Generating Map
##### Launch Cartographer
```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```
RViz should open up map geenration
##### Save the map (no extension needed to save the map)
```bash
ros2 run nav2_map_server map_saver_cli -f map_folder/map_name
```






