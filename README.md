# mavros_drone
#  How to Run

Clone the Repo in catkin workspace
```
catkin_make
roslaunch drone_sim forest_sim.launch 
roslaunch ground_control ground_control.launch
roslaunch goal_regulator regulator.launch
```

This would open 3 windows, the first window is the gazebo wheere we have our drone, then rvix that we would use to move drone to desired goal then dynamic reconfigure which would be used to dynamically change our controller and path planning parameters.
![alt text](/media/mavros.gif) 
