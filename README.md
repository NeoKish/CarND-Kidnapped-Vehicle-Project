## Kidnapped Vehicle Project 

This project is part of Udacity Self Driving Car Nanodegree which involves implementing a 2D particle filter in C++. In the build up to the project we were introduced to concepts of Localization, Bayesian filters, Markov 1D Localization and Particle filters.

### Project Introduction

The goal of the project is to implement a 2D particle filter in C++ to track a kidnapped vehicle transported to a new location with the help of the map of the location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.This project involves use of Udacity provided simulator (https://github.com/udacity/self-driving-car-sim/releases) to simulate the kidnapped vehicle environment.

![image](https://user-images.githubusercontent.com/66986430/121795158-c5da2500-cc2b-11eb-9162-2a667f372c3a.png)

### Running the Code

This file `src/main.cpp` contains the code that will actually be running your particle filter and calling the associated methods.

As requirement of the project, the code for src/particle_filter.cpp, and particle_filter.h had to be completed with initialization of the particles position with estimation of car's position from GPS input,prediction with the addition of control input such as yaw rate and velocity for all particles, updating the particles weights using map landmark position and feature measurements and implementation of resampling techique (technique use to improve the accuracy for localization) for drawing particles propotional to their weights. 

INPUT: values provided by the simulator to the c++ program

// sense noisy position data from the simulator

["sense_x"]

["sense_y"]

["sense_theta"]

// get the previous velocity and yaw rate to predict the particle's transitioned state

["previous_velocity"]

["previous_yawrate"]

// receive noisy observation data from the simulator, in a respective list of x/y values

["sense_observations_x"]

["sense_observations_y"]


OUTPUT: values provided by the c++ program to the simulator

// best particle values used for calculating the error evaluation

["best_particle_x"]

["best_particle_y"]

["best_particle_theta"]

//Optional message data used for debugging particle's sensing and associations

// for respective (x,y) sensed positions ID label

["best_particle_associations"]

// for respective (x,y) sensed positions

["best_particle_sense_x"] <= list of sensed x positions

["best_particle_sense_y"] <= list of sensed y positions

# Implementing the Particle Filter
The directory structure of this repository is as follows:

```
root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|
|___data
|   |   
|   |   map_data.txt
|   
|   
|___src
    |   helper_functions.h
    |   main.cpp
    |   map.h
    |   particle_filter.cpp
    |   particle_filter.h
```

### Inputs to the Particle Filter
You can find the inputs to the particle filter in the `data` directory.

#### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

### All other data the simulator provides, such as observations and controls.

> * Map data provided by 3D Mapping Solutions GmbH.
