Libraries used:
* numpy
* pygame

Install them using ```pip3 install pygame```

## Instructions to run
1. Open and run the script file corresponding to the A-Star algorithm : 'planner_v1.py'. You'll be prompted to enter inputs in the below given order.
2. Enter the sum of wall clearance and robot radius as a number in cm (for eg: 10)
3. Enter wheel velocities in RPM. You need to enter 2 values with a comma separation. (for eg: 6,6)
4. Enter the start state as a list of x, y, (both in cm) theta (in degrees) separated by commas (for eg: 10,10,0)
5. Enter the goal state as a list of x, y (both in cm) separated by commas (for eg: 10,10)
6. For points 2 and 3, if a non-number input is entered, the program keeps prompting till a number input is entered.
7. For point 4, the program keeps prompting till the start and goal states are entered in the proper manner suggested, and if the goal/start state is inside any of the obstacles' space.

## Output description
1. Upon successful execution of the algorithm if the goal can be reached, a pygame window opens up.
2. The green dots represent the visited nodes
3. The yellow dots represent the nodes in the open list at the time of algorithm termination
4. the red dots represent the optimal path taken.