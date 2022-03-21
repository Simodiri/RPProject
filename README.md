# 2D Lidar Matcher
I implented a 2D scan matcher, that takes in input 2D laser scans and, using ICP to find the transformation matrix between them.
In order to start it, follow this steps:
- First start the webctl from the repository of robot programming
- Start roscore, stage and map_server and rviz that will be used to take the laser_scans
- Change the fixed frame from "map" to "odom"
- Place the laser_scan
- Run "rosrun progetto progetto" and move your robot using either a joystick or the keyboard. It will return the transformations calculated with ICP and the final transformation from a laser_scan to another.
