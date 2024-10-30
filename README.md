# ENPM662 PROJECT-1 GROUP 10

## Overview
This project focuses on developing and simulating autonomous navigation and running teleop for the toycar(designed using Solidworks) using ROS2. The project encompasses teleoperation, visualization of lidar in RViz, and automated navigation within a Gazebo simulation environment.

## Setup
### 1. clone the repository
```bash
git clone https://github.com/siddhant-code/enpm662_project1
cd ~/enpm662_project1-main/project1_ws
```

### 2. Build the package
```bash
colcon build
```

### 3. Source the Setup File
After building, source the setup file to configure your environment:
```bash
source ~/project1_ws/install/setup.bash
```

### 4. Launching the competition world
To start the competition world gazebo simulation, run:
```bash
ros2 launch toycar competition.launch.py
```

### 5. Running RViz
In a new terminal, source the setup file and launch RViz for visualization:
```bash
source ~/project1_ws/install/setup.bash
ros2 launch toycar display.launch.py
```

### 6. Running teleoperation
Open another terminal, source the setup file, and run the teleoperation script to control the Toycar:
```bash
source ~/project1_ws/install/setup.bash
ros2 run teleop_script control_car
```

### 7. Launch gazebo world(empty world):
To start an empty Gazebo world for autonomous navigation execute:
```bash
ros2 launch toycar gazebo.launch.py
```

### 8. Running the automatic navigation:
```bash
ros2 run autonomous_script autonomous_control
```
