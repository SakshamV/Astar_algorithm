Libraries used:
* numpy
* pygame
* rospy

## Building the catkin workspace

1. Open a terminal window in the 'workspace' folder found in this directory.
2. If 'build' and 'devel' folders are present inside the 'workspace' folder, please delete those.
3. After making sure only 'src' folder is present inside the 'workspace' folder, run 'catkin_make' command in the terminal window to build the environment.
4. This is followed by executing 'source devel/setup.bash' command in the same terminal window.

## Launching the Gazebo simulation

1. Once the previous steps are done to build and source the workspace, launch the gazebo simulation using the following command in the same terminal:
'roslaunch turtlebot3_gazebo turtlebot3_p3.launch'
2. Upon successful execution of the previous step, gazebo opens up with the burger turtlebot spawned in a custom gazebo environment.

## Executing the path planner

1. With the Gazebo simulation running after successful execution of the previous steps, open a new terminal window in the 'workspace' folder and call the command 'source devel/setup.bash'
2. Run the planner node by executing the following command in the terminal which in turn calls the 'planner_v2.py' script :
'rosrun turtlebot3_gazebo planner_v2.py'
3. Upon execution of the planner_v2.py script, the user needs to enter the x and y position of the goal location in cm separated by a comma. (for eg: 500,0)
4. Once the goal location is entered, wait for a few seconds for the A* algorithm to solve for the optimal path.
5. Once the optimal path is computed, a pygame window opens up and draws the optimal path computed in red color.
6. NOTE: You need to close the pygame window, for the script to publish the optimal path's actions to Gazebo via a ROS publisher.
7. Upon closing the window you can see that the bot starts moving from its current location, following the optimal path computed till it reaches the goal.
