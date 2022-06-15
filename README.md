# 2022 REU on Smart UAVs
Auburn University 2022 REU on Smart UAVs

# The Project

###### Alec Pugh & Luke Bower

### UAV Path Planning using Aerially Obtained Point Clouds

#### Abstract
With the growing use of unmanned aerial vehicles (UAVs) for commercial and military operations, path efficiency remains an utmost concern for battery and time preservation. This paper presents a method for three-dimensional (3D) path planning using point clouds obtained from the USGS 3DEP (United States Geological Survey 3D Elevation Program) dataset via OpenTopography. The path itself is obtained using the A* algorithm, with additional modifications implemented to account for path smoothing, UAV size, and energy consumption. We also introduce a collision avoidance method using the precomputed data to account for unforeseen obstacles not rendered within the point cloud. The method presented is designed specifically for point clouds obtained via LiDAR (Light Detection and Ranging) scans from aircraft, where cavities may be present underneath the surface layer. Simulations and physical testing using waypoint transmission show the validity of this method. 


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
* Follow "Future Plans" section of midterm paper.

#
