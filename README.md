# PID Controller 

## Introduction:

This is the fourth  project in the second term of the Self Driving Car Nanodegree course offered by Udacity. In this project I implemented a PID controller to maneuver a vehicle around a simulated track. A [PID controller](https://en.wikipedia.org/wiki/PID_controller) continuously calculates an error value as the difference between a desired setpoint and a measured process variable and applies a correction based on proportional, integral, and derivative terms (sometimes denoted P, I, and D respectively) which give their name to the controller type. 

## PID Controller Implementation Details:
The P, I, and D parameters play different roles in the control system as described below:
* P generates a steering correction proportional to the cross track error (cte). Cross track error is the lateral distance between the vehicle and the reference trajectory. A large P results in the vehicle overshooting the track leading to oscillations
* I controls the drift in the vehicle. A misalignment in the car leads to drift which makes the car drive at an angle instead of in a straight line. To fix the effects of drift the I parameter collects the cte over time
* The D parameter dampens the overshooting caused by P. D is proportional to the changing rate of the cte

In this system the simulater does not have drift so I keep the I parameter to zero. Essentially this becomes a PD controller which is  strightforward to tune manually as explained below.

* I first assume the D parameter to be zero to tune the P parameter of a P controller. As the steering angle must be between -1 to 1 the P parameter has a maximum value of 1.3 with a cte of 0.7598 obtained from the simulator. The steering angle is related to the cte with the following formula:
steering angle = -P * cte

Setting the values for the parameters P, I, D to 1.3, 0, 0 results in a wobbly vehicle as shown in [this] video. I progressively half the value of P to 0.65, 0.325, and 0.1625. For P value of 0.1625 the car still wobles but much less. So fix the p value to be 0.1625.

As mentioned before the D parameter dampens the wobles seen with the P parameter. I start with a value of 1.0 for D and I get this result. The wobles have considerably reduced but teh vehicle goes off the road after some time. I progressively increment the D value by 1 and at D equals 3 I get a system where the vehicle stays inside the track.

With these parameters (P = 0.1625, I = 0, D = 3.0) I also varied the throttle from 30 mph to 90 mph. The vehicle stays inside teh track for speeds between 30 mph to 60 mph. 

## Conclusions:
While these parameters keep the car on the track between 30 to 60 mph the car is still wobbly. To reduce the woble and also to run the car at higher speeds I should use an algorithem like Twiddle to automatically choose the parameter values.



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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

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
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
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
