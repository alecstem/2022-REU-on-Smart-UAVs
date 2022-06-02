# Week 4

## End Goal

* Create an algorithm that reads in two GPS points and creates the most efficient 3D path for a UAV to traverse the points given with regard to the time, distance, and energy usage from aerially obtained LiDAR scans and the ablity to change course mid-flight if the best path is obstructed.

## Progress Report
* Added primitive object detection / avoidance in the simulation using euclidian distance.
* Current avoidance procedure: sample best avoidance path at every waypoint. If object is encountered, move to avoidance path and then recalculate A*.
* Began working on presentation slides for next week.
* Was able to incorprate Bezier curves to the path to smooth it out.
* Was able to download limited number of Laz files from data bases to create a smooth process from end to end.

## Obstacles
* A handfull of bugs that made things function incorectly on the small scale.
* Still unable to access all of the US LiDAR data. 
* We have not been able to work together as well due to one of us having to be in isolation.

## How We Faced These Obstacles
* We worked to think through the tiny bugs to fix them.
* We keep in touch thorugh online chat to try and keep the project moving along.

## Future Objectives
* Get access to the rest of the LiDAR data.
* Fully seam together the whole project for a rough first verion.
* Finish the asigments for the midterm. 
* Start testing in real life.

## Weekly Evaluation
* This week was full of small steps that brought togther the bigger picture of our reasurch. We were able to get most of the small missing parts of our original idea implemented and fixed some of the bugs that came with it. While we were physically seprated we were able to continue to work well togehter and acomplish what we had set out to do for this week. 

<p align="center">
  <img src="https://github.com/alecstem/2022-REU-on-Smart-UAVs/blob/main/images/point%20cloud.png" width="300" height="200" >
</p>
