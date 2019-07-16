<h1 align="center">Path Planning</h1>

<p align="center">
   <image src="./result.gif">
      </p>

### Summary
Use path planning to program a simulated vehicle that safely navigates around a virtual highway driving at +-10 MPH of the 50 MPH speed limit. The car is able to receive localization, sensor fusion data (lidar, radar, GPS), and a built in map of waypoints. 

### Success Criteria
- Stay close to the speed limit without exceeding it
- Drive inside the lane lines, except when changing lanes, stay on central lane is ideal
- Avoid all collisions, including going too slow and being rear-ended
- Accelerate and decelerate smoothly within acceleration limit of 10 m/s^2 and jerk limit of 10 m/s^3 (passenger comfort)
- Change lanes safely when the leading car is moving slowly

### Technical Details

- Predictions\
This step generates trajectories of all candidate vehicles and choose the one with lowest cost for reference. This is done in a purely model based approach.

- Behaviour planner\
This step defines a set of candidate high level targets for the vehicle to follow (lane changes, slow down). Here, the behavior of the vehicle is hard coded due to being sufficient for this challenge. It leaves a 30 gap from the front vehicle, changes lanes when blocks by a slow moving vehicle, and slows down if lane change is not feasible.

- Trajectories generation\
for every possible high level targets, a percise path to follow will be computed. This step is done using the spline library function based on xy coordinates. The path generated is ensured to be continuous. The upside of this approach is that it generates accurate path since the xy coordinates are more accurately estimated than the frenet coordinates. However, this approach does not guarantee a minimized sum of third derivative (total jerk). However, this approach does work well with the success criteria of this project and is simple to implement and computationally inexpensive.

- Trajectories cost ranking\
for each trajectory a cost will be derived (depending on feasibility, safety, legality, comfort and efficiency) and the trajectory with the lowest cost will be chosen. In this project, the specific cost value components how near the trajectory is from the nearest predicted vehicle, how much time it takes to reach the candidate point, how large is the acceleration and jerk, and a bit on whether we have to change lane. The overall aim of this model is to minimize risk by driving rather conservatively. 

### Data

- Main car's localization Data (No Noise)\
["x"] The car's x position in map coordinates\
["y"] The car's y position in map coordinates\
["s"] The car's s position in frenet coordinates\
["d"] The car's d position in frenet coordinates\
["yaw"] The car's yaw angle in the map\
["speed"] The car's speed in MPH

- Previous path data given to the Planner\
["previous_path_x"] The previous list of x points previously given to the simulator\
["previous_path_y"] The previous list of y points previously given to the simulator

- Previous path's end s and d values\
["end_path_s"] The previous list's last point's frenet s value\
["end_path_d"] The previous list's last point's frenet d value

- Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)\
["sensor_fusion"] A 2d vector of cars and then those cars' unique ID, xy position in map coordinates, xy velocity in m/s, and sd position in frenet coordinates. 

### Try it Yourself

Download [simulator](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2). If the binary file is not executable on linux or mac, enter the following in command line:
```shell
sudo chmod u+x {simulator_file_name}
```

1. Clone repo
2. `mkdir build && cd build`
3. `cmake .. && make`
4. `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

### Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

