In order to add aruco markers on to a robot in gazebo following approaches are tried:
* The marker is tried to be added as texture to a link. The material tag of the gazebo for urdf is really poorly documented. It is uncertain that we can do this approach at all. (FAILED FOR NOw)
* The marker is tried to print on the surface of a .dae mesh in blender. The result is more promissing. The marker can be seen in the rviz but Gazebo(7) can't read the textured .dae files created by blender. Maybe can be tested on later versions of Gazebo (FAILED FOR NOW)  
* The markers are drawn as urdf, brute force method. (WORKED)
