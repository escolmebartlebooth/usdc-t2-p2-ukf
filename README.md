# Unscented Kalman Filter Project for Self-Driving Car Engineer Nanodegree Program

Author: David Escolme
Date: 11 June 2018

## Project Objectives

Create an implementation in C++ of an unscented kalman filter using sensor fusion to estimate the state of a moving object of interest with noisy lidar and radar measurements so that measurement estimates meet the following target error:
* Position <= 0.09 and 0.10 RMSE in X and Y directions respectively
* Velocity <= 0.40 and 0.30 RMSE in X and Y directions respectively

[//]: # (Image References)

[image1]: NIS.PNG "NIS Values"
[image2]: RMSE.PNG "RMSE Values"

## Dependencies - taken from course notes

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./UnscentedKF

**Please note that a parameter can be passed to ./UnscentedKF. If a parameter is passed then this should be either 1 (use RADAR data only) or any other value (use LASER data only). Only if no parameter is passed will both sensor's data be used**

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

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Explanation of the data flow for measurements

Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)

OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---

## Code Structure and Operation

* main: The 2 changes made here were to log out the RMSE values for convenience and to capture a command line parameter to use in choosing whether to process all, laser, radar data.

* ukf: Handler class for each measurement. After the base initialisation of the class (private and public variables for use in the algorithm), an initialisation step uses the first measurement to setup the state and covariance matrices. Subsequent steps call the prediction and update methods, using the measurement type to choose which update is called.

* tools: RMSE calculation and also the Jacobian Matrix calculation

## Results

The x_ state vector was initialised to the first measurement's x and y position and zero for the other 3 state variables. The P_ covariance matrix was initialised to the Identity Matrix.

Using classroom defaults for the process noise vector, the algorithm diverged and encountered numerical instability causing the program to halt. Once this was understood (it took some time...), I reduced the noise vector elements to 1 from 30.

Once this was done, the filter converged to the target RMSE values within about 300 steps. Using the calculated NIS values, it was shown that the chosen values had arrived at a consistent filter. The NIS graphs for each sensor can be seen below.

![alt text][image1]

## Discussion Points:

* normalisation of phi in RADAR data: The difference between 2 time periods is y = z - z_pred. This could lead to a phi of <-3.14 or >3.14 radians. It is important to normalise this angle to be -3.14 >= rho <= 3.14. This is achieved by adding or subtracting 2*pi radians from the calculated phi difference value until the value falls between these limits.

* Use of only LASER or only RADAR data: From the 2 graphs below, it can be seen that when using one or other sensor in isolation, the filter converges to a higher error value. Only when combined does the convergence fall into the tolerance values for the project. Each sensor has its own inaccuracies. When combined, the overall effect is to 'average' each sensors deficiencies to arrive at a more accurate assessment of the state of the item being tracked. In this case, neither sensor individually tracks the curved path of the bicycle accurately.

![alt text][image2]

* Protecting against division by zero in the creation of sigma points is important as otherwise numerical instability will occur

* The first frame of data and parameter tuning. From the kalman filter, using the Identity Matrix for initial P_ was shown to be a good initialisation point. For the process noise variables (longitudinal acceleration in m/s^2 and yaw acceleration in rad/s^2) the initial values - as noted above - caused system failure. As the target vehicle was a bicycle, an acceleration of 30 either longtitudinal or yaw, would seem to be too high in reality anyway. Choosing a much lower value - one which was closer to a more realistic physical value - made sense.
