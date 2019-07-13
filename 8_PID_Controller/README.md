<h1 align="center">PID Controller</h1>

<p align="center">
 <a><img src="result.gif"></a>
</p>

### Summary

Programmed a basic PID (proportional integral derivative) mechanism to make a car drive itself on a simulated road with curves. This is a model-based approach to behavior cloning, which is a more data-based approach. However, this control loop feedback cycle still does quite well on staying on the lane. Despite thrust of the vehicle is programmed to be constant even when is it on curves, the vehicle is very good at always keeping itself on the track.

### Pipeline / Technical Detail

P component - "P stands for proportional. It is calculated as the product of p coefficient and cross track error. It lets the car steer proportionally to the car's distance from the CTE. A bigger P, the quicker for the car to get to target posiiton but the bigger the motional oscillation. 

D component - "D" stands for differential. It is calculated as the product of d coefficient different component of CTE. It serves as a way to let the car approach the center line of trajectory without oscillations and excessive jerk. 

I component - "I" stands for integral. It is calculated as the product of i coefficient and the sum of all CTE combined. The i coefficient takes a small value (0.0001 in my code) since the integral of cte is relatively large. The proportional control causes a bias over a long period of time and avoid the car to get in the exact trajectorry (systematic bias) and the integral term eliminates it. 

### Try it Yourself


**Dependencies**

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

**Basic Build Instructions**

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

### Success Criteria

The simulated autonomous vehicle has to stay on the lane thoughout the track. 
