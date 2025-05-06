# About the project.
- This project focuses on developing a tag-based autonomous docking system for the TurtleBot3
(burger) robot using ROS2 and Gazebo simulation. The goal is to enable the robot to autonomously navigate and
dock at a specified location using visual markers, specifically AprilTags.

## How to start the code.
- Initialize turtlebot3 model (burger with camera module in example) to use in simulation by printing following command in console.

   `export TURTLEBOT3_MODEL=burger_cam`

    If you want for this model to be chosen every time you open new terminal, you can use this command.

   `echo 'export TURTLEBOT3_MODEL=burger_cam' >> ~/.bashrc`

- Launching gazebo simulation.

  `ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py`

- Activation of Rviz for LIDAR based map creating and rqt for detection node work observing.
  
  `rqt`
 
  `ros2 launch turtlebot3_bringup rviz2.launch.py`

- Launching Apriltag detection node.

  `ros2 run turtlebot3_autorace_detect  detect_apriltag`

  After launching this node, in rqt go to pluggins > visualisation > image view  and select /detect/image_output/compressed to see when robot is            detecting apriltag and distance to it.

- Launching docking node.

  `ros2 run turtlebot3_autorace_detect apriltag_docking`
  
  This node will make robot automatically change orientation and position in order to dock near the apriltag.

- You can use autonomous obstacle avoidance node for robot to move by itself, or teleopt node to control your robot with keyboard.

  `ros2 run turtlebot3_gazebo turtlebot3_drive`

  `ros2 run turtlebot3_teleop teleop_keyboard`

# Video demonstration of the project.

https://youtu.be/s3iBAwgRvVw?si=_8hIQ4uBsvoEs5rw
