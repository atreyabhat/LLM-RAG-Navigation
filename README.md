# LLM-Navigation

![image](https://github.com/user-attachments/assets/c13f3ed2-ccef-4316-a906-eb07543e6987)


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
RViz should open up with map generation
##### Save the map (no extension needed to save the map)
```bash
ros2 run nav2_map_server map_saver_cli -f <map_folder/map_name>
```

#### Fixing DDS and Nav2 params
##### Fix DDS
```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
```
Add below line to .bashrc
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

##### Fix nav2 params
1) Go to
   ```bash
   cd /opt/ros/humble/share/turtlebot3_navigation2/param
   ```
2) Open waffle.yaml as admin
   ```bash
   sudo gedit waffle.yaml
   ```
3) Replace the robot_model_type: "differential" under amcl:ros__parameter with
   ```bash
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
   ```

#### Navigating Turtlebot3
1) Launch the desired simulation with the saved Map
   ```bash
   ros2 launch turtlebot3_gazebo <launch file.py>
   ```
2) Launch the navigation
   ```bash
   ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=<map_folder/map_name.yaml>
   ```
3) Provide 2D pose estimate in RViz (Approximate position and orientation)
4) Provide Nav2 Goal to navigate the robot to goal

#### Using LLM
1) Launch simulation and navigation for the turtlebot3 as mentioned above.
2) Launch the waypoint parser
   ```bash
   ros2 run waypoint_generator waypoint_parser
   ```
3) Launch the LLM interface
   ```bash
   ros2 run llm llm_publisher
   ```
   Follow the URL on the terminal which opens the web-based interface to provide user input to the robot.







