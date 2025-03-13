# amr_trajectory
This project focuses on Trajectory Visualization and Storage for AMR Navigation using ROS 2. It automates trajectory tracking in RViz and enables saving trajectory data in CSV formats. The system includes two C++ ROS nodes: one for publishing and saving trajectory data and another for reading and visualizing it. Ideal for AMRs in manufacturing. 

# AMR Trajectory Manager

## 1. Setup the Workspace

Navigate to the `amr_trajectory_manager` folder and build the package:


cd ~/amr_trajectory_manager
colcon build
source install/setup.bash


## 2. Launching the Simulation Environment

To start an empty world in Gazebo with the TurtleBot3:


ros2 launch turtlebot3_gazebo empty_world.launch.py


## 3. Moving the TurtleBot3 Autonomously

Run the autonomous movement node:


ros2 run amr_trajectory_manager turtlebot3_autonomous


## 4. Launching the Trajectory Saver and Visualizer

Run the trajectory logger node, specifying the directory to save trajectory data:


ros2 launch amr_trajectory_manager trajectory_logger.launch.py trajectory_directory:=<folder_directory>


## 5. Saving the Trajectory Data

Call the service to save the trajectory with a specified filename and duration:


ros2 service call /save_trajectory amr_trajectory_msgs/srv/SaveTrajectory "{filename: 'trajectory.csv', duration: 30.0}"


(Change the filename and duration as needed.)

## 6. Visualizing the Trajectory

Launch RViz2:


rviz2


Then:

- Add a **MarkerArray**
- Change **Fixed Frame** to `odom`
- Set the topic to `/trajectory_marker`

## 7. Stopping the Trajectory Logger

To stop the trajectory saver and visualizer node, use:


Ctrl + C


This will save the trajectory data to a CSV file.

## 8. Reading and Following the Saved Trajectory

Launch the trajectory reader node with the saved trajectory file:


ros2 launch amr_trajectory_manager trajectory_reader.launch.py trajectory_file:=/home/amith/trajectory1.csv


(Change the trajectory file path to match your saved file.)

## 9. Visualizing the Saved Trajectory

Launch RViz2 again:


rviz2


Then:

- Add a **MarkerArray**
- Change **Fixed Frame** to `odom`
- Set the topic inside **MarkerArray** to `custom_trajectory_markers`

Note : If you not seen any marks in rviz (dont close rviz) stop trajectory reader node and launch again  it will shows now

This completes the process of moving, saving, and visualizing the AMR trajectory using ROS 2.

