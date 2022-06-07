# 2022 REU on Smart UAVs
Auburn University 2022 REU on Smart UAVs

# The Project

###### Alec Pugh & Luke Bower

### 3D UAV Path Planning using Aerial/Incomplete Point Clouds

<p align="center">
  <img src="images/demo.gif" width="700" height="400" >
</p>

### General Process

* Download a point cloud in .las format from OpenTopography.
* Convert .las to a gridded DEM image of 1m per pixel.
* Run A* on every height layer to generate the optimal flight path for each crossection.
* Add buffer to obstacles using BFS to account for drone size. (or another way to smooth the line)
* Simulate UAV moving through path
* Convert pixel position to GPS way point and send this information to UAV.
##### Todo
* Improve collision avoidance
* Finish API integration to further automate the process.
* Real-life testing
* Write paper

#
