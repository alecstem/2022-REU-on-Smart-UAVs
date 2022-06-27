# Week 4

## End Goal

* Create an algorithm that reads in two GPS points and creates the most efficient 3D path for a UAV to traverse the points given concerning the time, distance, and energy usage from aerially obtained LiDAR scans and the ability to change course mid-flight if the best path is obstructed.

## Progress Report
* Added primitive collision detection/avoidance in the simulation using point cloud data and euclidian distance.
* Current avoidance procedure: sample the best avoidance path at every waypoint. If an object is encountered, pivot to the avoidance path and then recalculate the best path.
* Added path smoothing using Bezier curves. ~We were using an additional distance calculation to initially make the path smoother without Bezier, but we were able to remove that and it makes our program run a lot faster~ <-- path without additional sqrt looks strange.
* Can now download a controlled number of .laz files from databases.
* Began working on presentation slides for next week.

## Obstacles
* Sometimes the smoothened path will clip through buildings due to the interpolation. Working on a fix.
* Weird bugs with avoidance calculations that result in some points being infinity. Working on a fix.
* Still unable to access USGS LiDAR data automatically through API.
* We have not been able to work together due to one of us having to be in isolation.

## Future Objectives
* Work more on collision avoidance.
* Fix the path smoothing issues.
* Improve the speed of the program.
* Get access to the rest of the LiDAR data automatically.
* Finish the midterm assignments.
* Start testing in real life.

## Weekly Evaluation
* This week was full of small steps that brought together the bigger picture of our research. We were able to get most of the small missing parts of our original idea implemented and fixed some of the bugs that came with it. While we were physically separated we were able to continue to work well together and accomplish what we had set out to do for this week. 
