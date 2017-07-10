# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Implementation

**Model Predictive Control (MPC)** is an advanced method for process control which relies on dynamic models of the process. Different from previously implemented PID controller, MPC controller is able to anticipate future events and can take control actions accordingly. Indeed, future time steps are taking into account while optimizing the current time step.

### The Model

The MPC controller framework consists of the following key components:

- **Trajectory**: It is taken into consideration during the optimization. It consists of parameters of a number of time steps **N** spaced out by a time **dt**. The number of optimized variables is direct proportional to **N**, therefore it must be considered with computational constraints.

- **Vehicle Model**: A set of equations that describe system behavior and update time steps. In this project, I used a simplified kinematic model (aka. bicycle model) with 6 coefficients:
  - **x**: car position (x-axis)
  - **y**: car position (y-axis)
  - **psi**: car's heading direction
  - **v**: car's velocity
  - **cte**: cross-track error
  - **epsi**: orientation error

  Vehicle model update equations are implemented at lines 66-71 in *MPC.cpp*.

- **Constrains**: They are necessary for model actuators' response. In this model the following constraints are set:
  - **Steering**: ranged from [-25&deg;, 25&deg;]
  - **Acceleration**: ranged from [-1, 1]

- **Cost Function**: The whole control process is based on optimization of the cost function. Usually the cost function is made of the sum of different terms. Besides the main terms derived from reference values such as cross-track **cte** or heading error **cpsi**, other regularization terms are also introduced to ensure the smoothness of the controller response.

  The cost function is implemented at lines 45-64 in *MPC.cpp*.

### Trajectory Parameters

Timestep Length **N** and Elapsed Duration **dt** are rudimentary parameters in the optimization process. The prediction horizon **T = N &times; dt** is computed during the optimization. These two parameters were tuned with the following rule of thumbs:

- Large **dt** results in less frequent actuations, which can cause difficulties to follow a continuous reference trajectory (aka. discretization error).
- Large **T** can be benefit to the control process, however predicting too far in the future is not practical and meaningful in real-world scenarios.
- Large **T** and small **dt** lead to large **N**. As mentioned above, the number of optimized variables is direct proportional to **N**, therefore it also leads to higher computational cost.

In this project, these parameters are empirically set with a trial-and-error manner. Visual inspection is used to observe vehicle behaviors in the simulator. The final parameters are: **N = 10**, **dt = 0.1**, for **T = 1s** in the future.

### Polynomial Fitting
The polynomial fitting algorithm is referred to this method in the codebase of [Julia Math](https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L684-L725). It is implemented at lines 51-66 in *main.cpp*.

The method of mapping a standard vector to an Eigen vector is referred to this [forum post](https://forum.kde.org/viewtopic.php?f=74&t=94839#p194926). It is implemented at lines 107 and 108 in *main.cpp*.

### MPC with Latency
In order to mimic real driving conditions where the vehicle actuates the commands instantly, a 100 milliseconds latency delay is introduced before sending the data message to the simulator. It is implemented at line 174 in *main.cpp*.

---
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
