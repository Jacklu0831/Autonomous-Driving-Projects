<h1 align="center">Extended Kalman Filter</h1>

### Summary
Utilized kalman filter technique to estimate the state of a moving object of interest with noisy lidar and radar measurements. Estimated parameters are x position, y position, velocity in x, velocity in y. The lidar measurements are in red, radar measurements in blue, and predicted state in green.

<h4 align="center">Path 1</h4>
<p align="center">
<image src="./result1.gif">
</p>
 
<h4 align="center">Path 2</h4>
<p align="center">
<image src="./result2.gif">
</p>

### Technical Details

Kalman filter and extended Kalman filter fused the measurements from both radar and lidar together through a prediction measurement cycle. Knowing the state of the vehicle, kinematic formula is able to predict the future state of the car given elapse time. Then with the noise given of both lidar and radar, the measurements from both sensors would be used to future update the predicted state and use it for the next prediction. 

The extended Kalman filter is used for the radar update while normal Kalman filter is used for the lidar update, "extended" makes it mathematically more rigorous with the use of taylor expansion and jacobian matrix. However, the idea of updating measurement is the same.

**Input**: values provided by the simulator to the c++ program\
["sensor_measurement"]  =>  the measurement that the simulator observed (either lidar or radar)

**Output**: values provided by the c++ program to the simulator\
["estimate_x"]  <=  kalman filter estimated position x\
["estimate_y"]  <=  kalman filter estimated position y\
["rmse_x"]  <=  root mean squared error between estimation and ground truth of x\
["rmse_y"]  <=  root mean squared error between estimation and ground truth of y\
["rmse_vx"]  <=  root mean squared error between estimation and ground truth of vx\
["rmse_vy"]  <=  root mean squared error between estimation and ground truth of vy

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

