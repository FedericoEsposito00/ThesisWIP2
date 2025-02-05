<-- HOW TO LAUNCH THE SIMULATIONS -->

Push wall task:
1) Open a terminal and run "roslaunch uav-cable-suspended-robots-ros-pkg push_wall.launch"
2) Once Gazebo opens, unpause the simulation
3) Wait until the joint controllers are loaded (indicated by the terminal writing "0" repeatedly)
4) Press "enter" on the previous terminal to bring the arms to their initial position
5) Once the arms have reached their position, they can be controlled with the joystick
6) Open another terminal and run "rosrun hierarchical_ctrl_pkg sensors_hierarchical_ctrl_node"
7) Wait until the lift off is completed; the drone will start hovering in place
8) Open another terminal and run "rosrun hierarchical_ctrl_pkg publish_trajectory_node 0"
9) Once the trajectory is completed, the robot will push against the wall
10) Open another terminal and run "rqt"
11) Open the Message Publisher plugin and create a publisher for /licasa1/activate_control
12) The topic requires 4 terms: the first one is set to 1 to activate the force control, to 0 otherwise;
	the second one is the absolute value of the desired force;
	the third one is the desired angle between the reaction force and the heading direaction (ex: 3.14 to push forward with the robot);
	the fourth one is always set to 1 (if set to 0 makes the robot follow the trajectory in reverse)
	
Grasp object task:
1) Open a terminal and tun "roslaunch uav-cable-suspended-robots-ros-pkg grasp_obj.launch"
2) Once Gazebo opens, unpause the simulation
3) Wait until the joint controllers are loaded (indicated by the terminal writing "0" repeatedly)
4) Press "enter" on the previous terminal to bring the arms to their initial position
5) Once the arms have reached their position, they can be controlled with the joystick
6) Open another terminal and run "rosrun hierarchical_ctrl_pkg sensors_hierarchical_ctrl_node"
7) Wait until the lift off is completed; the drone will start hovering in place
8) Open another terminal and run "rosrun hierarchical_ctrl_pkg publish_trajectory_node 1"
9) Once the trajectory is completed, the robot will be a few centimeters away from the cube
10) Press the 3rd button of the joystick to activate the world-based control for the arms and the shared-control algorithm for the drone
11) Move the arms and the drone to approach the cube
12) Once the end effectors are on the sides of the cube, press the 1st button of the joystick four times to close the arms around the cube
13) Move the arms upwards to lift the cube
14) Manipulate the cube

<-- HOW TO USE THE JOYSTICK -->

The sticks of the joystick move the reference frame that the end effectors try to follow. 
The joystick must be set to analog.
From the point of view of the drone, the x axis of the reference frame points forward, the z axis points upwards and the y axis points left.
The left stick of the joystick controls the movements in the y-z plane; the right stick of the joystick controls the movements along the x axis.
The 1st button (X button on the logitech joystick) decreases the distance between the arms;
The 2nd button (A button on the logitech joystick) increases the distance between the arms;
The 3rd button (B button on the logitech joystick) changes the arms' control mode, changing the value of "ref state".
When "ref state = 0" the movement of the arms doesn't affect the movement of the drone;
When "ref state = 1" the movement of the arms also controls the drone through the shared-control algorithm.


