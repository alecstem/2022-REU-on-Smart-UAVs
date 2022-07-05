# Week 8

## End Goal

* Create an algorithm that reads in two GPS points and creates the most efficient 3D path for a UAV to traverse the points given concerning the time, distance, and energy usage from aerially obtained LiDAR scans and the ability to change course mid-flight if the best path is obstructed.

## Progress Report
* Used Selenium and OpenTopography API to remotely download all USGS files after inputting search polygon.
* Created code to parse .json file outputted from API to make sure datasets actually contained search polygon (API returns some datasets that don't contain any points for some reason)
* Modified collision avoidance to perform horizontal movement in most cases due to more success when testing.
* Finished the poster.
* Started working on the final draft for the paper.
* Started working on the final presentation.

## Obstacles
* Polishing off the code.
* Getting the collision avoidence to figure if an obstacle is big or not to change avoidance precidures.

## Future Objectives
* Finish the final deliverables
* While our process works in simulation. Real-life testing and the continuation of testing different factors to improve optimality should be tested. 
* Our process could be further by testing out different number of allowed points for the path to allow for the drone to fly faster.
* Add more advanced forms of collision avoidance.


## Weekly Evaluation
* This week was great! As we finished up the process that we have been devloping over the last few weeks and got some very promising results we look forward to seeing our reasurch have real life applications and effects. Over all the idea that we had in the start has now come into existance and we are more than satisifed with it. 
