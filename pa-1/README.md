
# Programming Assignment 1
By Julien Blanchet, 10/2/2019

### Setting Up
1. `cd catkin_ws`
1. `catkin_make`
1. `source devel/setup.bash`
1. `roscore`

### Running
Note: no need to manually run `rosrun turtlesim turtlesim_node`, it'll be launched automatically as part of the launch file.
* **Task #1**: `roslaunch homework1 task1.launch`
* **Task #2**: `roslaunch homework1 task2.launch`
* **Task #3**: You can choose several shapes! Feel free to edit the json to configure the path for the turtle to follow.
  * `roslaunch homework1 task3.trapazoid.launch`
  * `roslaunch homework1 task3.star.launch`
  * `roslaunch homework1 task3.spiral.launch`
  


### Sources Consulted
* Class notes
* For getting python to print to stderr [https://stackoverflow.com/questions/5574702/how-to-print-to-stderr-in-python](https://stackoverflow.com/questions/5574702/how-to-print-to-stderr-in-python)
    * Ultimately not used
* For using transforms: [http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28Python%29](http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28Python%29)