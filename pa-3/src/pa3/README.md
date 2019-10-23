# Programming Assignment 3

By Julien Blanchet, 10/26/2019

## Setting Up

1. `cd pa-3`
2. `catkin_make`
3. `source devel/setup.bash`
4. `roscore`
5. `chmod +x src/pa3/scripts/pa3.py` (if not using WSL)

## Running

* `roslaunch pa3 pa3.launch`
  
## Troubleshooting

* `ERROR: cannot launch node of type [map_server/map_server]: map_server`
  * Run `sudo apt-get install ros-melodic-map-server` (or the equvilant if you have ros kinetic)

## Sources Consulted

* Class notes / materials
* Information on publishing visualizations to rviz: [https://github.com/cse481sp17/cse481c/wiki/Lab-12:-Creating-Custom-Visualizations-in-RViz-using-Markers](https://github.com/cse481sp17/cse481c/wiki/Lab-12:-Creating-Custom-Visualizations-in-RViz-using-Markers)
