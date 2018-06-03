# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

[![Alt text](https://img.youtube.com/vi/5mLn6PW2Nw8/0.jpg)](https://www.youtube.com/watch?v=5mLn6PW2Nw8)


## Development
1. I used the code from MPC quizzes as baseline
https://github.com/udacity/CarND-MPC-Quizzes/blob/master/mpc_to_line/solution/MPC.cpp
2. This was **kinematic equation** used from the class and the code

| Equation | Code  |
| ----- | ------------- |
| ![](./assets/equation.png?raw=true) | `fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);`<br>`fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);`<br>`fg[1 + psi_start + t] = psi1 - (psi0 - v0 * delta0 / Lf * dt);`<br>`fg[1 + v_start + t] = v1 - (v0 + a0 * dt);`<br>`fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));`<br>`fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) - v0 * delta0 / Lf * dt);`<br> |

3. This the code for **state, actuators and update equations**. I modified the actuator change rate from 1 to 1000 to get a slower rate of change and allow car to drive smoothly. 1000 seemed to work better than a class recommended 500
````
    for (int t = 0; t < N; t++) {
      fg[0] += 1*CppAD::pow(vars[cte_start + t], 2);
      fg[0] += CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    // Minimize the use of actuators.
    // Minimize change-rate.
    for (int t = 0; t < N - 1; t++) {
      //multiply by a huge value to do controlled steering.
      fg[0] += 1000*CppAD::pow(vars[delta_start + t], 2);
      fg[0] += CppAD::pow(vars[a_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (int t = 0; t < N - 2; t++) {
      fg[0] += 1*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }
````
4. **timestamp duration (dt) and timestamp length (N)**
I used N = 10 and dt=0.12 which results in T = 120 milliseconds. Also I set the latency as 0.1 Sec (100 milliseconds) which was closer to calculated T. My values were further reinforced by some discussions from slack channel that this was a good value to consider. And they were giving me better results than N = 25 and dt = 0.05 from MPC quizzes. 
5. **Polynomial Fitting and MPC Preprocessing**
I converted the world coordinates to vehicle coordinates with following function
````
void map2car(double px, double py, double psi, const vector<double>& ptsx_map, const vector<double>& ptsy_map, Eigen::VectorXd & ptsx_car, Eigen::VectorXd & ptsy_car){
  for(int i=0; i< ptsx_map.size(); i++){
    double dx = ptsx_map[i] - px;
    double dy = ptsy_map[i] - py;
    ptsx_car[i] = dx * cos(-psi) - dy * sin(-psi);
    ptsy_car[i] = dx * sin(-psi) + dy * cos(-psi);
  }
}
````
This was then used to derive the cte and epsi
````
map2car(px, py, psi, ptsx, ptsy, ptsx_car, ptsy_car);
auto coeffs = polyfit(ptsx_car, ptsy_car, 3);
double cte = polyeval(coeffs, px);
double epsi = -atan(coeffs[1]);
````
6. **Model Predictive Control with Latency**
Since the coordinates where converted to vehicle orientation, I just used the original kinematics equation and replaced the `dt` with `latency`, `a` with `throttle`, `delta` with `steer_angle` and set the other values to zero and later initialized the state.
*Note: Even after doing this, my car was making wrong turns. It turns out, I had to set the `steer_angle` to nwgative before initializing. This was discovered by following some slack discussions.*
````
double p_px = 0 + v * cos(0) * latency;
double p_py = 0 + v * sin(0) * latency;
double p_psi = 0 - v / Lf * steer_angle * latency;
double p_v = v + (throttle * latency);
double p_cte = cte - 0 + (v * sin(epsi) * latency);
double p_epsi = epsi + p_psi;
state << p_px, p_py, p_psi, p_v, p_cte, p_epsi;
````

---

## Dependencies

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
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
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
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

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

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
