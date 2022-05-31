# Week 3

## End Goal

* Create an algorithm that reads in two GPS points and creates the most efficient 3D path for a UAV to traverse the points given with regard to the time, distance, and energy usage from aerially obtained LiDAR scans and the ablity to change course mid-flight if the best path is obstructed.

## Progress Report
* We were able to create a 3D model of our 2D path for visualization.
* Then we converted the A* algorithm from 2D to 3D by adding in the evaluation of 26 nodes (all of the possible 3D directions for a cubic grid).
* We started working on simulations to have a proof of concept and way to see the best path play out in a real time enviroment.
* We were able to convert our cartesian points to latitude and longitude to send to the drone as waypoints.
* Briefly started to optimise our program.
* We worked on our paper.

## Obstacles
* We faced plently of bugs this week, some even taking a day and a half to fix.
* It took a while to convert A* from working in a 2D environment into 3D.
* We are having problems downloading a limited amount of files from the API and servers to be able to create a fully functinal process.
* Re-evaluating the A* when we find an obstacle in the way and making the algorithm able to run fast enough to fix the path while in flight.

## How We Faced These Obstacles
* We did a lot of debugging and creating small fixes mostly just creating more of a standard in our code for the way things move in the program.
* We had to do a lot of math and testing to get the nodes to work and weigh correctly in the new A*.

## Future Objectives
* Get the downloading working and create more of a together system for the whole process.
* Get the object dectection and avoidance working in a simulation.
* Speed up the computational time of the program.
* Start to add in more factors for A* to have more defined constraints..
* Get a good simulation working for the program.

## Weekly Evaluation
* This week was very good. We faced many more problems than the week before but we were able to keep our heads up and persever through the trials and still make some progress every day. We have a much better understanding of how our program should work and operate leading us to both be able to work on seperate parts and it still come together smoothly. Overall, this week was very good and we made solid progress and hope that the trend of these past few weeks continue. 

<p align="center">
  <img src="https://github.com/alecstem/2022-REU-on-Smart-UAVs/blob/main/images/point%20cloud.png" width="300" height="200" >
</p>
