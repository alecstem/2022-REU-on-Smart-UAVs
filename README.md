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
   
