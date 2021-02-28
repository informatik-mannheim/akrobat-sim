# Simulating the ROS Navigation Stack

#### Creating the map

1. launch the gazebo.launch (start rviz on default open_rviz:=true/false)
roslaunch hexapod gazebo.launch

2. start map building
roslaunch hexapod hexapod_gmapping.launch

3. start teleop --> Robot needs to move around and feed map with info
roslaunch hexapod hexapod_teleop.launch

### Saving the map
4. save the map to a file path
rosrun map_server map_saver -f /home/mh/catkin_ws/src/hexapod/hexapod_mapping/map

### Loading the map
Close all previous terminals and run the following commands below.  Once loaded, use rviz to set navigation waypoints and the robot should move autonomously.

5. launch gazebo.launch
roslaunch hexapod gazebo.launch

6. start navigation with the saved map
roslaunch hexapod hexapod_navigation.launch

11. In rviz, send a few `2D Navigation Goals` (click on `2D Nav Goal and click/drag to set position/orientation) and watch the robot autonomously navigate to the goal.

Close all terminals, you are done with mybot navigation.
