# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

[//]: # (Image References)

[image1]: ./readme_file/lane_change.gif "Lane Change"
[image2]: ./readme_file/following.gif "Following without Collision"
[image3]: ./readme_file/Goal.gif "Safe Distance Achieved"


### Basic Description
The project is based on the couser provided from Udacity. Original description of the simulator and data input can be found [here](https://github.com/udacity/CarND-Path-Planning-Project/blob/master/README.md)

### Path Points Smoothing (Starting from line 282)
The path points smoothing is achieved by adding evenly 30m-spaced points ahead of the starting reference (in Frenet) and then use spline function to smoothly interpolate waypoints bewteen these sparse points.

It is worth mentioning that a coordinates transfromation was conducted for the simplicity of the coding and spline calculation (line 343 transform to car coordinates: 0 degree is the front; line 394 transform it back to the original coordinates system)

### Lane Changing
the car is able to change lane safely when there is a slow vehicle in the front. It tends to change back to center line if the center lanes are safe.
![alt text][image1]

the car is able to follow behind a car.
![alt text][image2]

the car is able to run at least 7.6 miles without any violations such as large jerks or collisions.
![alt text][image3]
### Reflection: 
The lane change part includes many parameters depending on the safe SPACING to ensure the car-following and lane-changing safety, which is determined based on many rounds of tests through the simulation.

I didn't use the finite mahcine or cost functions to determine the best driving behavior among multiple trajectories, since the lane changing in highway environment is easy enough and could be done through some simple logics.
