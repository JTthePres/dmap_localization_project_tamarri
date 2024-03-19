DMap based localization project for Robot Programming exam (*master degree Artificial Intelligence and Robotics given from LaSapienza*)

# Assignment 
Write a program that listens a Grid-Map from a map server, extracts the obstacles (occupied cells), and populates a Distance Map with the world coordinates of these points.  The program should output a transform between the map and odom, similar to what a localizer should do.  The program works by computing the transform between the current scan and the map, using ICP.  It subscribes to a topic /initialpose to set the initial position from rviz. Each time a new scan is received the program performs a new registration, starting from an intial guess. This initial guess is obtained by applying the odometry displacement to the estimate of the previous time instant
# Short Description
The project is structured in one package called **listener** that contains all the auxiliaries files provided during the course and the main node called **listener_node** .
This node subscribes to the three topics map(for receiving the map), initialpose(for receiving the initial pose setted by rviz) and base_scan (for receiving the laserScans).
Then It publish under the topic "oodom" mexages with the current odometry after the localization and the receiving of the initial pose.

# Execution
  For testing the project  a map and a  .world file are attached.
  For executing the simulation of the laser scanner use stageros.
  Commands:
  `rosrun map_server map_server map.yaml`
  `rosrun stage_ros stageros cappero***.world`
  