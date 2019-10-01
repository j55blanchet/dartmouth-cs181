
# Programming Assignment 1
By Julien Blanchet


### Setting Up
1. `cd catkin_ws`
1. `catkin_make` )
1. `source devel/setup.bash`
1. `roscore`

### Running
Note: no need to manually run `rosrun turtlesim turtlesim_node`, it'll be launched automatically as part of the launch file.
* **Task #1 (simplist implementation)**: `roslaunch homework1 task1_simple.launch`
* **Task #1**: `roslaunch homework1 task1.launch` (to specify radius, edit `src/homework1/launch/task1.launch`)


### Sources Used
* Class notes
* For getting python to print to stderr [https://stackoverflow.com/questions/5574702/how-to-print-to-stderr-in-python](https://stackoverflow.com/questions/5574702/how-to-print-to-stderr-in-python)
* For using transforms: [http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28Python%29](http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28Python%29)