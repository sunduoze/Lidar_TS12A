
TS12A Lite ROS2

1.How to build 3iRobotics lidar ros2 package:

2) Please select correct serial in “src\node.cpp”：/dev/ttyUSB0(default)
3) cd delta_2b_lidar
4) colcon build
5) source install/setup.bash
6) sudo chmod 777 /dev/ttyUSB0


2.How to run 3iRobotics lidar ros2 package:	
 2.1 run publish_node:
 	1)Open new Termial: 
 	source install/setup.bash
 	
 	2)ros2 run delta_2b_lidar delta_2b_lidar_node
	//rosrun delta_lidar delta_lidar_node or roslaunch  delta_lidar delta_lidar.launch(NOTE:please select correct serial in “launch\delta_lidar.launch”)
 2.2 run subscribe_node:
	1)Open new Termial: source install/setup.bash
	2)ros2 run delta_2b_lidar delta_2b_lidar_node_client    
	
3.How to run 3iRobotics lidar rviz:
	1)Open new Termial: 
	source install/setup.bash
	
	2)ros2 launch delta_2b_lidar delta_2b_lidar.launch.py serial_port:=/dev/ttyUSB0 



