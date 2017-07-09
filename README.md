# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

This project is a part of Udacity's Self-Driving Car Engineer
Nanodegree Program.  The project was cloned from Udacity
github repository.

main.cpp and MPC.cpp were edited to implement the Model Predictive Control.

## Implementation

### The Model
This Model Predictive Control (MPC) is implemented. This model simulates the actuator inputs, predicts the results from a trajectory, and selects a trajectory with a minimum cost at each time step. Then, use those actuator inputs, and use that new states and calculate a new trajectory. 

Six state variables are used in this project:

	x => the vehicle position at x-axis
	y => the vehicle position at y-axis 
 	psi => the vehicle yaw orientaiton
	v => the vehicle speed
	cte => cross track error
	epsi => the orientation (yaw) errors.

Two actuators (control inputs) used are:
	
	delta => steering angle
	a => throttle (acceleration/braking)

The state variables for a next time step can be calculated by:

	x[t+1] = x[t] + v[t] * cos(psi[t])*dt
	y[t+1] = y[t] + v[t] * sin(psi[t])*dt
	psi[t+1] = psi[t] + v[t]/Lf * delta[t]*dt
	v[t+1] = v[t] + a[t] * dt
	cte[t+1] = cte[t] + v[t] * sin(epsi[t])*dt
	epsi[t+1] = epsi[t] + v[t]/Lf * delta[t]* dt

Note that Lf is the distance from the front of the vehicle to the vehicle c.g.   
In this project, it is a constant value and set to 2.67.

dt is the timestep (see the next section).  


### Timestep Length and Elapsed Duration (N and dt)

The simulation runs pretty good with 30 MPH.  The vehicle runs at 50 MPH, but it goes closer to the curb when there is a large curve in the road.

With 30 MPH, the settings of N = 15 and dt = 0.05 seconds are used.
These values are determined by the trail and error.  They are picked by observing the cte values and ran the simulation to see if the vehicle runs successfully.

### Polynomial Fitting and MPC Preprocessing.

The waypoints (in main.cpp, they are defined as ptsx and ptsy) are obtained from the simulator in the global coordinate system.

Since the state is expressed in a vehicle coordinate system, the conversion was performed. [in main.cpp, lines 116-123.]

After transforming those points in the vehicle coordinate system, use the predefined Polyfit function to obtain the third order polynomial coefficents.  
Once the coeffients are otbained, create a reference line by specifiying a number of points (15) and the spacing (1.5) as x values.

### Model Predictive Control with Latency

In MPC, one of the contributiong factors of the latency is the actuator dynamics such as there is a time delay between when the actuator information (steering angle, throttle) are sent and when these information is actually received. In this project, it is assumed that there is a 100ms delay.
To avoid this problem, predict the state uses this latency delay (100ms), and then feed them into the solver. [in main.cpp, lines 108-112.]. 


## Simulation

The simulation runs with 30 MPH.  The vehicle can run at 50 MPH but it goes too close to the curb.
To successfully run the simulation, a lot of tunings were required.  And unfortunately, with the limited time available to complete this project, could not find the tunining parameters which work for any vehicle speed.  To be able to run with 30 MPH, added multipliers (weights) to a part of the cost function affecting steering and throttle [in MPC.cpp, diff_delta_mul = 10000 for steering, and diff_a_mul = 1000 for throttle.  Picking these numbers are also by trial and error with different speed.]


---
Below, the original comment when the project was cloned.

## Dependencies

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
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
