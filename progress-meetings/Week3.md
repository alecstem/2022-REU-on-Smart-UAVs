# Week 3

## End Goal

* Create an algorithm that reads in two GPS points and creates the most efficient 3D path for a UAV to traverse the points given with regard to the time, distance, and energy usage from parcial LiDAR scans and the ablity to change course mid flight if the best path is obstructed.

## Progress Report
* We were able to ceate a 3D modle of our 2D path for visulation of how it works on a by height baises.
* Then we addapted A* to be able to evaluate 26 nodes(all of the possible 3D directions for a cubic grid) to find the best path to move along diffrent heights.
* We started working on simulations to have a proof of concept and way to see the best path play out in more of a real time enviroment.
* We were able to convert our web points to the related GPS points to send to the drone as waypoints.
* Breifly started to optimise our program to reduce computational time.
* We also started to work on our final paper and get ideas written down for that along with a rough look at our introduction. 

## Obstacles
* We faced plently to bugs this week some even taking a day and a half to fix.
* We strugled to make the A* be able to correctly evaluate the 26 nodes and their weight.
* We are having problems downloading a limited ammount of files from the API and servers to be able to create a fully functinal process.
* Re-evaluating the A* when we find an obstical in the way and the program being able to run fast enough to fix the path while in flight.

## How We Faced These Obstacles
* We did a lot of debuging and creating small fixes mostly just creating more of a standard in our code for the way things move in the program.
* We had to do a lot of math and testing to get the nodes to work and weigh correctly in the new A*.

## Future Objectives
* Get the downloading working and create more of a together system for the whole process.
* Get the object dectection and avoidence working in a simulation.
* Speed up the computational time of the program.
* Start to add in more factors for A* to have more defined constraints..
* Get a good simulation working for the program.

## Weekly Evaluation
* This week was very good. We faced many more problems than the week befor but we were able to keep our heads up and persever through the trials and still make some progress every day. We have a much better understanding of how our program should work and operate leading us to both be able to work on seprate parts and it still come together smoothly. Overall, this week was very good and we made solid progress and hope that the trend of these past few weeks contine. 

<p align="center">
  <img src="https://github.com/alecstem/2022-REU-on-Smart-UAVs/blob/main/images/point%20cloud.png" width="300" height="200" >
</p>
