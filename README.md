[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

![](./record.gif)

---

### The project
The goal of this project is to design a proportional-integral-derivative (PID) controller in order to keep the center of the lane of a track of the Udacity simulator.

At each timestamp the controller receives from the simulator the car's lateral displacement from the center of the road.
Based on this error the controller computes a steer command that is then sent back to the simulator and actuated.

The tuning of the three PID parameters has been done manually and following these general guidelines:
* Increasing the proportional gain (Kp) reduces the vehicle time response but at the same time increases overshoot, also leading to instability.
* Increasing the integral gain (Ki) reduces the steady-state error but, if chosen too big, can lead to system's instability.
* Increasing the derivative gain (Kd) generally introduces stability, reducing the oscillatory behavior. Since it acts on the derivative of the error attention must be paid on this parameter: a huge delta error would lead to a spike in the control action.

Here is reported the final set of parameters:
* Kp = 0.10  (P)
* Ki = 0.004 (I)
* Kd = 1.4   (D)

### Reflections
* The car is able to perform the full track without accidents.
* Since an integral action is used, an anti wind-up system should also be considered to manage the cases when the control input saturates.
* Oscillations are generally always present. A more advanced control techniques (such as LQR or MPC) should be used here to integrate vehicle model and information ahead of the ego-vehicle (such as heading angle and road curvature)


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

