
# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Overview 
In this project, a PID controller is designed to keep the car in the middle of the road. [The simulator](https://github.com/udacity/self-driving-car-sim/releases) reports back the car's distance from the center of road, angle, and speed. The control output is steering angle of the car. At the end, the optimum parameters are extracted using the manual method. To spice things up, a second controller is added to increase the speed of the car whenever it is safe. A video of final system can be found [here](https://youtu.be/hFsznNKY0sg).

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

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
## Effect of each PID coefficient
### Proportional coefficient
The P controller pushes the steer toward the setpoint with the control force proportional to the error (hence the name). The sole P controller can cause overshoot and oscillation which for this problem means driving to the other side of the road and sea sickness for the passengers. On the good side, increasing this coefficient can lower the response time and less sensitive system. Keeping the P coefficient low, on the other hand, doesn't make a good controller because of high response time.
### Integral coefficient
The I controller accumulates the error during time and helps to eliminate the steady state error and bias in the system. Increasing this coefficient can lower the response time and eliminate the bias better, while sacrificing the stability (overshoot and oscillation) and increasing the settling time like the P controller. Regarding the problem in hand, since the simulator doesn't have any bias, this coefficient just gathers the error and causes instability and changes the controller output even when the car is in the middle of the road.
### Derivative coefficient
The D controller acts according to the error changing rate; this helps to decrease the controller output when the system is getting close to the setpoint; thus decreasing the overshoot and settling time; on the negative side though, this adds a damping effect to the system. In our system, the D controller can predict if the car is going to deviate from the center of the road, even when the error is small. 
## PID tuning approach
### Steering controller
Keeping the car in middle of the road can translate into a disturbance rejection problem. PID controller acts well in this scenarios because of the unknown nature of disturbance and complex dynamics of the car steering system.
The [manual method](https://en.wikipedia.org/wiki/PID_controller#Manual_tuning) is chosen for PID tuning for this project. First, all coefficients are set to zero. Then the P coefficient is increased (0.05 steps) until the car can follow the pass with some oscillation. The final value for the P part is 0.15. Since the car doesn't have any bias, tuning the I parameter is skipped. The D coefficients has major role in this controller; as it is mentioned before, a controller with stronger the derivative nature can predict if the car is going to deviate from the center of road even when the error is small. This helps to act earlier when the car approaches road turns. Also, this part of the controller helps to keep the overshoot and oscillation low. I increased this parameter with the steps of 0.5 and the final values is 1.5.
### Throttle controller
This controller track the speed setpoint set depending on the cross track error (CTE). The speed setpoint can increases up to 40 mph when the error is small and decreases down to 25 mph when the error is large. This behavior matches the human drivers reaction too. when the car wonder off the center, the driver tend to slow down and control the vehicle. For this controller, the P coefficient is small (0.1), so the car doesn't accelerate too fast. During tuning of the P, a bias is noticed in the system. To achieve the desired speed and stay close to it without any bias, integral coefficient is added (0.001) but kept small to avoid the overshoot. Since the P and I parts are set to small values, the overshoot is small, so the D part didn't have that much effect in this controller; therefore, it is set to zero.

## Future improvements
While Twiddle method introduced in the course may seem a more systematic approach, it works best when the system can run repeatedly without any setup. With the current system, I have to change the PID parameters, compile and run the program, fire up the simulator and observe the results. As a future improvement, it would be great to use a system which doesn't need that much setup. For instance, if the program could control the simulator, it would significantly help the process.


