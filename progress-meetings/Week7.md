# Week 7

## End Goal

* Create an algorithm that reads in two GPS points and creates the most efficient 3D path for a UAV to traverse the points given with regard to the time, distance, and energy usage from aerially obtained LiDAR scans and the ablity to change course mid-flight if the best path is obstructed.

## Progress Report
* Worked more on collision avoidance by introducing set of avoidance trajectories based on the type of obstacle encountered. (bigger object = horizontal movement preferred)
* Introduced energy into pathfinding algorithm by basing cost of node by distance so far to node, heuristic to end node, and now the energy required by the UAV to move to that neighboring node. ( F(n) = g(n) + h(n) + e(n) )
* Created a process that uses the API to find a database that contains the search polygon and downloads the .las file from the website automatically.
* Combined this with previous code to create a start to end process that uses all datasets available on OT.
* Made collision avoidance trajectories take into account the heading of UAV.
* Started testing different heuristics for statistical analysis.
* Adpted the ARDU waypoint process to speed up flight time to a faster but still under controle point.

## Obstacles
* Collision avoidance trajectories do not take into account the heading and pitch of the UAV yet.
* Keeping the simulated drone under controle and following the path but also being efficent and fast.

## Future Objectives
* Work more on collision avoidance.
* Fix the path smoothing issues.
* Improve speed of program.
* Get access to the rest of the LiDAR data automatically.
* Finish the midterm assignments.
* Start testing in real life.

## Weekly Evaluation
* 
