# Programming Assignment 3

By Julien Blanchet, 10/26/2019

This package defines a class that impelments the A* graph search algorithm on an occupancy grid. The package includes associated tests and is designed to be visualized using rviz.

## Setting Up

1. `cd pa-3`
2. `catkin_make`
3. `source devel/setup.bash`
4. `roscore`
5. `chmod +x src/pa3/scripts/pa3.py` (if not using Windows / WSL)

## Running

* `roslaunch pa3 pa3.launch`
* Launch `rviz` and ensure it has the following displays:
  * "Map" with topic of `/map`
  * "Marker" with topic of `/visualization_marker`
  * "Pose" with topic of `/pose_sequence`
  
## Troubleshooting

* `ERROR: cannot launch node of type [map_server/map_server]: map_server`
  * Run `sudo apt-get install ros-melodic-map-server` (or the equivalent if you have ros kinetic)

## Sources Consulted

* Class notes / materials
* [Information on publishing visualizations to rviz](https://github.com/cse481sp17/cse481c/wiki/Lab-12:-Creating-Custom-Visualizations-in-RViz-using-Markers)
* [Python documentation for named tuples](https://docs.python.org/2/library/collections.html#collections.namedtuple)
* [Python priority queue documentation](https://docs.python.org/2/library/queue.html)
* [Python A* algorithm tutorial](https://www.simplifiedpython.net/a-star-algorithm-python-tutorial/)
  * Note that my impelmentation style differs greatly. I was mostly looking for which data structure would be helpful - `PriorityQueue` in this case
* [A* on Wikipedia](https://en.wikipedia.org/wiki/A*_search_algorithm
)
  