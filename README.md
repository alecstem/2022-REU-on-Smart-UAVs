# 2022 REU on Smart UAVs
Auburn University 2022 REU on Smart UAVs

# The Project

###### Alec Pugh & Luke Bower

### 3D UAV Path Planning Using Aerially Obtained Point Clouds

#### Abstract
With the growing use of unmanned aerial vehicles (**UAVs**) for commercial and military operations,
path efficiency remains an utmost concern for battery and time preservation. This paper presents a
method for three dimensional (**3D**) path planning using point clouds obtained from the USGS 3DEP
(**United States Geological Survey 3D Elevation Program**) dataset via Open Topography. The
path itself is obtained using the A* algorithm, with additional modifications implemented to account for
path smoothing, UAV size, and energy consumption. We also introduce a collision avoidance method
using the precomputed data to account for unforeseen obstacles not rendered within the point cloud. The
method presented is designed specifically for point clouds obtained via LiDAR (**Light Detection and
Ranging**) scans from aircraft, where cavities may be present underneath the surface layer. Simulations
show the validity of this method.


<p align="center">
  <img src="images/demo.gif" width="700" height="400" >
</p>

### General Process
* Download a point cloud in .las format from OpenTopography.
* Convert .las to a gridded DEM image of 1m per pixel.
* Add buffer to obstacles using BFS to account for drone size. (or another way to smooth the line)
* Run adapted A* on every height layer to generate the optimal flight path for each crossection.
* Simulate UAV moving through path
* Convert pixel position to GPS way point and send this information to UAV.
* Start collision avoidence in real time on the simulation.

### About
This repo has two directorys that holds the code for the collision avoidance system and the full simulation not including the collision avoidance activated. To use the CAS code you simply need to change the name of the desired file in path_visulation.py and run this script(make sure the las file is in the same folder as the script). To use the full simulation you need to run main and enter in the start and end latitude and longitude points along with changing the download paths in main, your opentopography user name, your opentopography password, and in drone_testing.py change the cmd definition to your path. This repo also holds pictures of our process and the weekly progress reports.

##### Future Work
* While our process works in simulation. Real-life testing and the continuation of testing different factors to improve optimality should be tested.  
* Our process could be further by testing out different number of allowed points for the path to allow for the drone to fly faster. 
* Add more advanced forms of collision avoidance.


