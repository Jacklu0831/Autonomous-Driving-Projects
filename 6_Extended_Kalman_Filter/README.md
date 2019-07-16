<h1 align="center">Extended Kalman Filter</h1>

### Summary
Utilized kalman filter technique to estimate the state of a moving object of interest with noisy lidar and radar measurements. Estimated parameters are x position, y position, velocity in x, velocity in y. The lidar measurements are in red, radar measurements in blue, and predicted state in green.

#### Path 1
<p align="center" font-weight="bold">
<image src="./result1.gif">
</p>
 
#### Path 2
<p align="center" font-weight="bold">
<image src="./result2.gif">
</p>

### Steps

**INPUT**: values provided by the simulator to the c++ program\
["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)

**OUTPUT**: values provided by the c++ program to the simulator\
["estimate_x"] <= kalman filter estimated position x\
["estimate_y"] <= kalman filter estimated position y\
["rmse_x"] <= root mean squared error between estimation and ground truth of x\
["rmse_y"] <= root mean squared error between estimation and ground truth of y\
["rmse_vx"] <= root mean squared error between estimation and ground truth of vx\
["rmse_vy"] <= root mean squared error between estimation and ground truth of vy\

### Success Criteria

The green path should not deviate from the car's ground truth path. The error measurement is RMSE and if a set threshold (main.cpp) is exceeded, the warning message will show up on the simulator.  

### Try it Yourself

Simulator can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).
Once the install for uWebSocketIO is complete (refer to the [installation folder](./installation), the main program can be built and run by doing the following from the project top directory.

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `

## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

