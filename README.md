# About the project.
- This project focuses on developing a tag-based autonomous docking system for the TurtleBot3
(burger) robot using ROS2 and Gazebo simulation. The goal is to enable the robot to autonomously navigate and
dock at a specified location using visual markers, specifically AprilTags.

## How to start the code
- Initialize turtlebot3 model (burger with camera module in example) to use in simulation by printing following command in console.

`export TURTLEBOT3_MODEL=burger_cam`

 If you want for this model to be chosen every time you open new terminal, you can use this command.

 `echo 'export TURTLEBOT3_MODEL=burger_cam' >> ~/.bashrc`
