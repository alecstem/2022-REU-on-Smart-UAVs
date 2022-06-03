# Week 4

## End Goal

* Create an algorithm that reads in two GPS points and creates the most efficient 3D path for a UAV to traverse the points given with regard to the time, distance, and energy usage from aerially obtained LiDAR scans and the ablity to change course mid-flight if the best path is obstructed.

## Progress Report
* Added primitive object detection / avoidance in the simulation using point cloud data and euclidian distance.
* Current avoidance procedure: sample best avoidance path at every point. If object is encountered, pivot to avoidance path and then recalculate A*.
* Added path smoothing using Bezier curves.
* Can now download a limited number of .laz files from databases.
* Began working on presentation slides for next week.

## Obstacles
* Sometimes the path smoothing will clip through buildings due to the interpolation. Working on a fix.
* Still unable to access USGS LiDAR data automatically through API.
* We have not been able to work together due to one of us having to be in isolation.

## Future Objectives
* Work more on collision avoidance.
* Fix the path smoothing issues.
* Improve speed of program.
* Get access to the rest of the LiDAR data automatically.
* Finish the midterm assignments.
* Start testing in real life.

## Weekly Evaluation
* This week was full of small steps that brought togther the bigger picture of our research. We were able to get most of the small missing parts of our original idea implemented and fixed some of the bugs that came with it. While we were physically seperated we were able to continue to work well together and accomplish what we had set out to do for this week. 
