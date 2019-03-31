# Robot Localization implementation in ROS

Qiwei Yang



[![video](https://img.youtube.com/vi/P2CsBjCM1Qk/0.jpg)](https://www.youtube.com/watch?v=P2CsBjCM1Qk)


### Abstract

Robot localization is essential to solve robot path planning / navigation tasks. Planning under uncertainties, which are resulted from inaccurate controller, imperfect sensors, 
unexpected environments, etc, is a quite challenging. Accurate estimation of the robot states or poses, which localization aims to accomplish, is critical in decision making under these
uncertainty.
      
There are several powerful algorithms to estimate the robot location, two of which are covered in this project, Kalman Filter and Monte Carlo Localization. ROS community provides packages like AMCL and Navigation
to allow users to easily implement these algorithms in Gazebo simulator.  

## 1. Introduction

There are two key capabilities that a robot shall have when it moves in an environment: Localization and Mapping. Localization means the robot knows the environment or map in advance,
and need to accurately identify where it is continuously when moving, so that it knows if it is approaching the target location in the right direction. Mapping means the robot has no idea about
the environment in advance, but it knows its location and can continuously update the info with confidence, therefore it can map the environment simultaneously when moving. 
Simultaneous localization and mapping, SLAM for short, is a hot research area both in academia and industry. This project deals with localization problem only.    

Localization can be divided into three sub-problems : local localization, global localization, and the kidnapping. In the local localization, also
known as position tracking, the robot knows its initial pose and the problem is to keep track of the robot’s pose as
it moves. In global localization, the robot’s initial pose is unknown, and the robot tries to determine its pose relative
to the ground truth map; the uncertainty for this type of localization is greater than for local localization. In the
kidnapped robot problem, just like in global localization, the robot’s initial pose is unknown, however the robot maybe
kidnapped at any time and moved to another location of the map. Solving for the latter challenge also helps the robot
recover in the event that it loses track of its pose, due to either being moved to other positions, or even when the
robot miscalculates its pose. [reference](https://github.com/csosa27/RoboND-Localization-Project/blob/master/Where%20Am%20I.pdf)

## 2 Mainstream Filters: 

Basic kalman filter is usually used for linear systems with Gaussian noise, and extended kalman filter can be applied for non-linear systems. Monte Carlo method, also called particle filter, can be applied for non-linear systems easily. 
Particle filter is computationally more expensive than Kalman filter, but easier to implement and understand. 

### 2.1 Kalman Filter(KF) and Extended Kalman Filter(EKF)

KF in a nutshell: 

A KF is an optimal estimation algorithm mainly invented by Rudolf E. Kalman. Common applications include guidance and navigation systems, computer vision systems and signal processing. It has two major applications:  

1. The variables of interest can only be measurement indirectly.  
2. Several measurements are available from various resources but are subject to noise.  
  
Specifically in this project, there are two main uncertainties existing in the localization. One is the control or motion uncertainty, the other measurement noisy. In practice, these uncertainties follow the gaussian distribution pretty well.
KF makes use of this property and consider both of them to estimate the location. See the figure below: 

![kalmanfilter](./images/kf.png)  

[MATLAB reference](https://www.mathworks.com/videos/understanding-kalman-filters-part-5-nonlinear-state-estimators-1495052905460.html)  

"Predicted state estimate" mentioned in the figure is generated from robot motion through controller/actuator, which has uncertainty for sure. For example, the robot may
have unwanted wheel slip, motor inaccuracy, etc. In addition, all measurements have noises, and not perfect. Usually considering their errors generate better estimation than considering only one of them along. 


### 2.2 Monte Carlo Localization method (also called Particle filters)

[Matlab MCL](https://www.mathworks.com/help/robotics/ug/monte-carlo-localization-algorithm.html)  
Particle filters
Monte Carlo localization algorithm similar to Kalman Filters estimates the posterior distribution of a robot’s position and orientation based on sensory information but instead of using Gaussians it uses particles to model state.

The algorithm is similar to KF where motion and sensor updates are calculated in a loop but MCL adds one additional step: a particle resampling process where particles with large importance weights (computed during sensor updates) survive while particles with low weights are ignored.

In the MCL example below all particles are uniformly distributed initially. In the following update steps the particles that better match the predicted state survive resulting in a concentration of particles around the robot estimated location.

### 2.3 Filter comparison and summary


## 3 Simulation environment

### 3.1 Model creation through urdf/xacro

### 3.2 Packages Used

In order to launch the mobile robots, a ROS package was
created. The robot package structure was designed as shown
below. Udacity Bot Package:  

* meshes
* urdf
* worlds
* launch
* maps
* rviz
* config

This robot package, along with the Navigation Stack
and AMCL packages were crucial for a complete simulation
of a mobile robot performing and successfully solving the
localization problem.

### 3.3 Parameters

To obtain most accurate localization results, several parameters were added, tested and tuned. The parameter values
obtained for the udacity bot were tuned in an iterative process to see what values worked best. In the AMCL node,
the most prominent parameters were the min particles and max particles which were set to 10 and 200, respectively.
These, tuned the accuracy of the localization process. An increase of particles would mean an increase in accuracy,
however, it would also have impact on computational efficiency, making processing slower. (See Table ?? for detailed parameter specifications) Several other parameters
were tuned in the different config files. The transform tolerance, inflation radius, robot radius, and obstacle range
were obtained after several iterations of testing and tuning. Increasing the inflation radius would have an impact on the
costmap while detecting obstacles and their distance related to the robot. Whereas robot radius represents the radius
of the robot as it relates to its environment. Meaning that having a lower robot radius value would increase chances of
it getting stuck around obstacles, and a higher value would prompt the robot to think it was bigger than the space it
had to pass by. Albeit briefly summarized here, the ROS Wiki Page) provides in-depth descriptions of each and every
one of the parameters used for this project as well as other parameters that can be explored further. See TABLE 1 and
TABLE 3 for detailed parameters specifications used for the udacity bot

Table 1  
Global and Local Costmap Parameters: Udacity Bot

|Parameter   |Value | Impact   |
|:---|:---|:---|
| global frame  | map   |   |
| robot base frame  |robot footprint   |   |
|update frequency   | 15.0   |   |
|publish frequency   |15.0    |   |
|width   |20.0   |   |
|height   |20.0   |   |
|resolution   |0.05   |   |
| static map  |true   |   |
| rolling window  |false   |   |


Table 2    
AMCL and Other Parameters: Udacity Bot

| Parameters  | value  | 
|:---|:---|
|min particles   | 10   |
|max particles  |  200 |
| obstacle range  | 2.5  |
| raytrace range  |   |
|transform tolerance   |   |
|robot radius   |   |
| inflation radius  |   |
| holonomic robot  |   |
|yaw goal tolerance   |   |
|xy goal tolerance   |   |
|sim time   |   |
|meter scoring   |   |
|max vel x   |   |
|max vel y   |   |
| max vel theta  |   |
| acc lim theta  |   |
| acc lim x  |   |
|acc lim y   |   |
| controller frequency  | 15  |
|gdist scale   |   |
| pdist scale   |   |
|   |   |
|   |   |
|   |   |
|   |   |




[reference 1](https://medium.com/@fernandojaruchenunes/udacity-robotics-nd-project-6-where-am-i-8cd657063585)

[Other resource, kalman filter in matlab](https://blogs.mathworks.com/headlines/2016/09/08/this-56-year-old-algorithm-is-key-to-space-travel-gps-vr-and-more/)