# Week 7

## End Goal

* Create an algorithm that reads in two GPS points and creates the most efficient 3D path for a UAV to traverse the points given concerning the time, distance, and energy usage from aerially obtained LiDAR scans and the ability to change course mid-flight if the best path is obstructed.

## Progress Report
* Worked more on collision avoidance by introducing a set of avoidance trajectories based on the type of obstacle encountered. (bigger object = horizontal movement preferred)
* Introduced energy into pathfinding algorithm by basing the cost of the node by distance so far to node, heuristic to end node, and now the energy required by the UAV to move to that neighboring node. ( F(n) = g(n) + h(n) + e(n) )
* Created a process that uses the API to find a database that contains the search polygon and downloads the .las file from the website automatically.
* Combined this with previous code to create a start-to-end process that uses all datasets available on OT.
* Made collision avoidance trajectories take into account the heading of the UAV.
* Started testing different heuristics for statistical analysis.
* Adapted the ARDU waypoint process to speed up flight time to a faster but still under control point.

## Obstacles
* Collision avoidance trajectories do not take into account the heading and pitch of the UAV yet.
* Keeping the simulated drone under control and following the path but also being efficient and fast.

## Future Objectives
* Work more on collision avoidance.
* Fix the path smoothing issues.
* Improve the speed of the program.

## Weekly Evaluation
* This week was very successful. We were able to finish the complete process and smooth it out this week and this then allowed for us to do the various types of testing for some comparison data. We were also able to get an almost fully functioning form of primitive collision avoidance working this week. Overall everything that we set out to accomplish has been finished and this next week we plan to test out some new things with our process to try and find the most optimal path possible. We hope to finish these next few weeks as strong as this week.
