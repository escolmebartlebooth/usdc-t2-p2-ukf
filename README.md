# Extended Kalman Filter Project for Self-Driving Car Engineer Nanodegree Program

Author: David Escolme
Date: 30 May 2018

## Project Objectives

Create an implementation in C++ of an extended kalman filter using sensor fusion to estimate the state of a moving object of interest with noisy lidar and radar measurements so that measurement estimates meet the following target error:
* Position <= 0.11 RMSE (x and y direction)
* Velocity <= 0.52 RMSE (x and y direction)

[//]: # (Image References)

[image1]: RMSE.PNG "RMSE Values"

## Dependencies - taken from course notes

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

**Please note that a parameter can be passed to ./ExtendedKF. This should be either 1 (use RADAR data only) or 2 (use LASER data only). All other values will use both data inputs.**

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

* FusionEKF: Handler class for each measurement. An initialisation step uses the first measurement to setup the state and covariance matrices. Subsequent steps call the prediction and update methods, using the measurement type to choose which update is called.

* kalman_filter: This class contains the prediction and 2 update methods.

* tools: RMSE calculation and also the Jacobian Matrix calculation

## Results

Using classroom defaults for the State X and State Covariance Matrix P, the algorithm converges to be within tolerance within:
* Position: Measurement 36
* Velocity: Measurement 339

The initialisation values were:

X = [Mxi, Myi, 0, 0] so the first measurement of position and zero velocity
P (diagonals) = [1, 1, 1000, 1000] so resonable confidence in position and very low confidence in initial velocity state

To improve the time taken to converge on the RMSE targets for Velocity, I experimented lowering the P values for velocity confidence and found that - roughly - a value of 10 for both x and y velocity directions allowed an earlier convergence on target for both datasets to around 260 measurements.

## Discussion Points:

* normalisation of rho in RADAR data: The difference between 2 time periods is y = z - z_pred. This could lead to a phi of <-3.14 or >3.14 radians. It is important to normalise this angle to be -3.14 >= rho <= 3.14. This is achieved by adding or subtracting 2*pi radians from the calculated phi difference value until the value falls between these limits.

* Use of only LASER or only RADAR data: From the 2 graphs below, it can be seen that when using one or other sensor in isolation, the filter converges to a higher error value. Only when combined does the convergence fall into the tolerance values for the project. Each sensor has its own inaccuracies. When combined, the overall effect is to 'average' each sensors deficiencies to arrive at a more accurate assessment of the state of the item being tracked.

![alt text][image1]

* Protecting against division by zero in the Jacobian Matrix Hj: To avoid division by zero, a low limit value of 0.001 was set and would be used if the value of the input parameter to the matrix fell below that value.

* The first frame of data: The velocity converges slowly to the tolerance level. The project hinted that better handling of the first frame may improve the algorithm. I tried, as discussed above, reducing the P matrix value for covariance for velocity in both the X and Y directions. This appeared to improve convergence by about 50 measurements. I suspect this is because a value of 1000 provides a very broad range to iterate on given the maximum likely velocity range of the item being tracked and by reducing the covariance, the algorithm can more quickly arrive at a more accurate estimate.

