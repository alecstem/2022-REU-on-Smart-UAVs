# 2022 REU on Smart UAVs
Auburn University 2022 REU on Smart UAVs

# The Project

###### Alec Pugh & Luke Bower

### Shortest Path from Satellite Imagery for Height Restricted Operation
* We talked about this.
    1. Use a method to process satellite imagery.
    1. Create a grid with likely obstacles based on altitude.
    1. Compute shortest path w/ added buffer depending on drone size.
    1. Send coordinates to drone.
    2. Then take this data and compair it to a topographic map to then evaluate best hight.
    2. Would need to account for changes in the imagery like construction.
    2. Put together images to form full path and have a designated distance per grid/pixel(paper said 2 meters per).
    2. Might need a way to pick the color for correct path/land.
    3. Possibly take into consideration the chagns in elevation for strain if we intend on relaying that information to not just air but also ground.(just a thought) 
    4. for the city scape we need to add in the max heights for if a building or structure exceeds that
    5. should also probly have a live feed from the drone to the computer and possibly a visual of where it is located along that path?(this can be done from the internal gps in the drone itself)


use a conversion of lidar distance to unit of measure and possibly use that for both distance travled and controling the direction of the drone.
Use Open Topgraph website to get lidar scan for given region of gps then download a laz or las file(laz will have to be unziped to be a las) that is ran through a c++ library to convert that data to a set for x,y and z corridnates that can then be used in something like an if check to find out what heights have the most frequent occurence to then compair the path of least resistance. The x and y will be evaluaed based on the FAA* algorithems top lets say 3 paths to then create an evaluation of the paths that is formed from a binary conversion of the data to find where the obstructions will be. Once that path is created they will be sent to the drone in a step by step method by the use of waypoints along the way to trace the most efficent path (this will be achived by either gps siginals or by the use of conversion of speed and distance to get the time of flight for each step). 

   
