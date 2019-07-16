<h1 align="center">Particle Filter</h1>

<p align="center">
    <image src="./result.gif">
</p>

### Summary

In this project, my vehicle has been kidnapped. However, with noisy GPS data (initial positional estimation), sensors (lidar+radar), and a built-in map, I was able to use a 2D particle filter technique and localize my car within centimeters. 

### Technical Details

This was the most mathematically heavy and the most interesting project due to having to wrap my head around recursive bayesian estimation and markov chains. With a discrete number of particles (10 used for the gif above) each carrying position and weight measures that updates every prediction/measurement, the algorithms uses their individual weights to localize the vehicle sequentially by only using the previous state of the vehicle and particles for predicting the next. [Here](https://medium.com/@jonathan_hui/tracking-a-self-driving-car-with-particle-filter-ef61f622a3e9) is an awesome article that explains the problem in 1D instead for anyone who is interested.

### Running the Code

Download simulator [here](https://github.com/udacity/self-driving-car-sim/releases).

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO. Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following:

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

Alternatively use the scripts:

1. ./clean.sh
2. ./build.sh
3. ./run.sh

### Success Criteria

Qualitatively, the blue circle under the vehicle should never leave it. Quantitatively, the `max_translation_error` and `max_yaw_error` should never exceed values specified in `src/main.cpp`. If exceeded, warnings would show up typically in the first 10 seconds of the simulation.
