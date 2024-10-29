#ENPM662 PROJECT-1 GROUP 10

#clone the repository
git clone https://github.com/siddhant-code/enpm662_project1
cd ~/enpm662_project1-main/project1_ws

#Build the package
colcon build
source install/local_steup.bash

#Launch competition world:
ros2 launch toycar competition.launch.py

#Run the teleop (open new terminal):
source install/local_steup.bash
ros2 run teleop_script control_car

#Launch gazebo world(empty world):
ros2 launch toycar gazebo.launch.py 

#To run automatic navigation:
ros2 run autonomous_script autonomous_control
