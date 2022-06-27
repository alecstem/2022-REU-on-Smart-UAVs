# Week 2

## End Goal

* Create an algorithm that reads in two GPS points and creates the most efficient 3D path for a UAV to traverse the points given concerning time, distance, and energy usage.

## Progress Report

* Took point cloud data from LiDAR scans, converted it into a grid, and wrote code to separate layers based on height.
* Implemented A* on each height layer and created a visualization.
* Took in two GPS points and found the database where the LiDAR scans are located.
* Wrote conversion code for all types of points used.
* Performed literary analysis on our topic and adjacent topics.

## Obstacles

* Initially wanted to use satellite imagery but had to switch to point clouds obtained from LiDAR scans to retrieve height data.
* Finding a resource that would let us obtain point cloud data for anywhere we wanted was a challenge.
* Creating the images from a 2D array of ones and zeros took a while due to matching issues.
* Finding the conversion factors for the LiDAR x, y, and z points into GPS coordinates.
* Access files from API and the servers they are located on.

## How We Faced These Obstacles

*  We found OpenTopography, a website that houses LiDAR datasets from the USGS 3DEP project.
*  To solve the conversion factors we had to research the way that LiDAR receives data and the way it converts the values from GPS to the web version of WGS84 Web Mercator [epsg: 3857] to GPS and vice versa. We also had to figure out the relationship of the Z-axis. We found that it is based on the NAVD88 system and relates equivalently to meters above sea level so we take our ground point as a base number and subtract that from all others to find the change in height from the measurement in meters.
* As we got started on this project we wanted to make it all happen in a streamlined way. For us, that means small file sizes and all in one place. To achieve this we needed to work with the database that holds the LiDAR data and their API. However, their API is not very developed so we are actively working with Rose Pearson in New Zealand to create a library to solve these issues.

## Future Objectives

* Make A* smarter (add optimization, height checks, etc.)
* Finish OpenTopography API integration
* Integrate an energy consumption factor in path planning.

## Weekly Evaluation

* This week went phenomenal. We worked very diligently to formulate a plan and outline of our project finding large flaws with the original idea and working to find solutions. Once that was done we started to split our days into half implementation of code and the other half literary analysis. This allowed for us to move at a comfortable pace of seeing progress and addressing rising questions as well as how we can improve the research that has already been performed. We were able to have some of the very basic elements of our program run and this led us to a better idea of what is possible with our plan. We had some bigger problems arise that we are still having to address, however, overall this week was very productive and we hope to continue to have weeks like this as we progress.


<p align="center">
  <img src="https://github.com/alecstem/2022-REU-on-Smart-UAVs/blob/main/images/point%20cloud.png" width="300" height="200" >
  <img src="https://github.com/alecstem/2022-REU-on-Smart-UAVs/blob/main/images/before_astar_auburn.gif" width="300" height="200" >
  <img src="https://github.com/alecstem/2022-REU-on-Smart-UAVs/blob/main/images/before.png" width="300" height="200" >
  <img src="https://github.com/alecstem/2022-REU-on-Smart-UAVs/blob/main/images/after.png" width="300" height="200" >
  <img src="https://github.com/alecstem/2022-REU-on-Smart-UAVs/blob/main/images/cross_auburn_buffer2.gif" width="300" height="200" >
  <img src="https://github.com/alecstem/2022-REU-on-Smart-UAVs/blob/main/images/lats_and_longs_to_location_or_database_name.png" width="1200" height="200" >
  
  
</p>
