

# Create static transform publisher
#  Tell the program that the sonar is located at z=2 compared to base_link, rotated at pi/4
rosrun tf static_transform_publisher 0 0 2 0.78 0 0 base_link sonar_link 100

# Note: in rviz, you need to add tf in left-hand display column

# Add laser link facing a different directions
rosrun tf static_transform_publisher 0 2 2 -0.78 0 0 base_link laser_link 500


# Adding an unconnected link (say there's a second robot)
rosrun tf static_transform_publisher 0 2 2 -0.78 0 0 robo2_base_link robo2_laser_link 500



# Starting a robot in a different simulator
rosrun stage_ros stageros -d /opt/ros/.../maze.world