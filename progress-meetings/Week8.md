# Week 8

## End Goal

* Create an algorithm that reads in two GPS points and creates the most efficient 3D path for a UAV to traverse the points given concerning the time, distance, and energy usage from aerially obtained LiDAR scans and the ability to change course mid-flight if the best path is obstructed.

## Progress Report
* Used Selenium and OpenTopography API to remotely download all USGS files after inputting search polygon.
* Created code to parse .json file outputted from the API to make sure datasets included the search polygon (API returns some datasets that don't contain any points for some reason)
* Modified collision avoidance to perform a horizontal movement in most cases due to more success when testing.
* Finished the poster.
* Started working on the final draft for the paper.
* Started working on the final presentation.

## Obstacles
* Polishing off the code.
* Getting the collision avoidance to figure out if an obstacle is big or not to change avoidance procedures.

## Future Objectives
* Finish the final deliverables
* While our process works in simulation. Real-life testing and the continuation of testing different factors to improve optimality should be tested. 
* Our process could be further by testing out different numbers of allowed points for the path to allow for the drone to fly faster.
* Add more advanced forms of collision avoidance.


## Weekly Evaluation
* This week was great! As we finished up the process that we have been developing over the last few weeks and got some very promising results we look forward to seeing our research have real-life applications and effects. Overall the idea that we had at the start has now come into existence and we are more than satisfied with it. 
