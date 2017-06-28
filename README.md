# PID Controller 

## Introduction

This is the fourth project in the second term of the Self Driving Car Nanodegree course offered by Udacity. In this project, I implemented a PID controller to maneuver a vehicle around a simulated track. A [PID controller](https://en.wikipedia.org/wiki/PID_controller) continuously calculates an error value as the difference between a desired setpoint and a measured process variable and applies a correction based on proportional, integral, and derivative terms (sometimes denoted P, I, and D respectively) which give their name to the controller type. 

## PID Controller Implementation Details
The P, I, and D parameters play different roles in the control system as described below:
* **P** generates a steering correction proportional to the cross track error (cte). Cross track error is the lateral distance between the vehicle and the reference trajectory. A large P results in the vehicle overshooting the track leading to oscillations. A small P results is fewer oscillations but also means it is slower to respond to corrections
* **I** controls the drift in the vehicle. A misalignment in the car leads to drift which makes the car drive at an angle instead of in a straight line. To fix the effects of drift the I parameter collects the cte over time
* The **D** parameter dampens the overshooting caused by P. D is proportional to the changing rate of the cte

In this system, the simulator does not have drift so I keep the I parameter to zero. Essentially this becomes a PD controller which is straightforward to tune manually as explained below. P and D move in opposite directions and the trick is to find optimal values for these parameters to keep the vehicle on the track.

I first assume the D parameter to be zero to tune the P parameter of a P controller. As the steering angle must be between -1 and 1 the P parameter has a maximum value of 1.3 with a cte of 0.7598 obtained from the simulator. The steering angle is related to the cte with the following formula:

*steering angle = -P * cte*

Setting the values for the parameters P, I, D to 1.3, 0, 0 results in a wobbly vehicle as shown in [this](https://youtu.be/E94Acxrwkz0) video. I progressively half the value of P to 0.65, 0.325, and 0.1625. For P value of 0.1625 the car still wobbles but much less. So, I fixed the P value to be 0.1625.

As mentioned before the D parameter dampens the wobbles seen with the P parameter. I start with a value of 1.0 for D and I get [this](https://youtu.be/rhh67k21dIk) result. The wobbles have considerably reduced but the vehicle goes off the road after some time. I progressively increment the D value by 1 and at D equals 4 I get a system where the vehicle stays inside the track. See the final result [here](https://youtu.be/gy5sFYA7r6U).

With these parameters (P = 0.1625, I = 0, D = 4.0) I also varied the speed from 30 mph to 70 mph. The vehicle stays inside the track for speeds between 30 mph to 50 mph. 

## Conclusions
While these parameters keep the car on the track between 30 to 50 mph the car is still wobbly. To reduce the wobble, I tried decreasing P and increasing D further but for these values while the wobble decreases the vehicle goes on the curb a bit because it is not quick enough to respond to corrections. [Here](https://youtu.be/LHHxrWOUdaA) is a video for parameters (P = 0.0813, I = 0, D = 4.0). An algorithm like Twiddle to automatically choose the optimal parameter values will reduce the wobble and also enable to run the car at higher speeds.

---

## Dependencies for Windows 10

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run ./install-ubuntu.sh.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

## Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`


