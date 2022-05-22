# Week 2

## End Goal
* Create an algorithem that reads in two GPS points and creates the most efficent 3D path for a UAV to terverse the points given in reguards to the time, distance, and energy useage.

## Progress Report
* Took point cloud data from LiDAR scans, converted it into a grid, and wrote code to seperate layers based on height.
* Implemented A* on each height layer and created a visualization.
* Took in two GPS points and found the database that the LiDAR scans are located.
* Wrote conversion code for all types of points used.
* Performed literary analysis on our topic and adjacent topics.

## Obstacles

* Initially wanted to use satellite imagery but had to switch to point clouds obtained from LiDAR to retrieve height data.
* Finding a resource that would let us obtain point cloud data for anywhere we wanted was a challenge.
* Creating the images from a grid of ones and zeros took a while due to matching issues.
* Finding the conversion factors for the LiDAR x, y, and z points into GPS cordinates.
* Accessing files from API and the servers they are located on.

## How We Faced These Obstacles
* We ...

## Objectives

* Make A* smarter (add optimization, height checks, etc.)
* Finish OpenTopography API integration
* Intgrate an energy consumption factor in the path planning.

## Weekly Evaluation
* This week went ...


<p align="center">
  <img src="https://github.com/alecstem/2022-REU-on-Smart-UAVs/blob/main/images/point%20cloud.png" width="300" height="200" >
  <img src="https://github.com/alecstem/2022-REU-on-Smart-UAVs/blob/main/images/before_astar_auburn.gif" width="300" height="200" >
  <img src="https://github.com/alecstem/2022-REU-on-Smart-UAVs/blob/main/images/before.png" width="300" height="200" >
  <img src="https://github.com/alecstem/2022-REU-on-Smart-UAVs/blob/main/images/after.png" width="300" height="200" >
  <img src="https://github.com/alecstem/2022-REU-on-Smart-UAVs/blob/main/images/cross_auburn_buffer2.gif" width="300" height="200" >
  <img src="https://github.com/alecstem/2022-REU-on-Smart-UAVs/blob/main/images/lats_and_longs_to_location_or_database_name.png" width="1200" height="200" >
  
  
</p>
