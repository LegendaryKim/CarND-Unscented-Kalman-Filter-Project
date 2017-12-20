# Unscented Kalman Filter Project Starter Code

Self-Driving Car Engineer Nanodegree Program

## Overview

I utilized an Unscented Kalman Filter with C++ to estimate the state of a moving object of interest with noisy lidar and radar measurements.
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)
This repository includes two files that can be used to set up and intall [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.


## Depandancies

* cmake >= 3.5
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
* gcc/g++ >=5.4

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Build Instructions

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./UnscentedKF (Outputfile)

"Outputfile" are required to print the outcomes of simulation including RMSEs and NIS.

## Input/Output

INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurment that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

## Results

[//]: # (Image References)

[image1]: ./Output/Output1.png "Output 1"
[image2]: ./Output/Output2.png "Output 2"
[Image3]: ./Output/NIS.png "NIS"

![alt text][image1]
![alt text][image2]

Here is the [output video](.Output/SensorFusion_UnscentedKalmanFilter.mov)

## Rubrics

### Compiling

#### Your code should compile.
Yes. It compils with ```cmake``` and ```make``` as mentioned in the "Build Instruction".

### Accuracy

#### For the new version of the project, there is now only one data set "obj_pose-laser-radar-synthetic-input.txt". px, py, vx, vy output coordinates must have an RMSE <= [.09, .10, .40, .30] when using the file: "obj_pose-laser-radar-synthetic-input.txt"
The results of RMSE are
* Dataset 1: [0.0612, 0.0859, 0.3302, 0.2135]
* Dataset 2: [0.0654, 0.0609, 0.6189, 0.2556]

### Follows the Correct Algorithm

#### Your Sensor Fusion algorithm follows the general processing flow as taught in the preceding lessons.
The code mainly follows the sensor fusions and Unscented Kalman filter algorithms with the codes in ```../src```

#### Your Kalman Filter algorithm handles the first measurements appropriately.
The initialization step for the state vectors and covariance matrices are applied in [ukf.cpp](../src/ukf.cpp) from line 92 to 154.

#### Your Kalman Filter algorithm first predicts then updates.
The prections step are at [ukf.cpp](../src/ukf.cpp) from line 176 to 275 and the updates are placed at the same file from line 279 to 407.

#### Your Kalman Filter can handle radar and lidar measurements.
Different scheme are utilized in the first measurements (from line 101 to 135) and updates steps (from line 279 to 315) in [ukf.cpp](../src/ukf.cpp).  

### Code Efficiency

#### Your algorithm should avoid unnecessary calculations.
I optimized the code without duplicated calculations. 



CF. I tuned the process noise parameters ```std_a = 0.5``` and ```std_yawdd = 0.5``` by NIS.
![alt text][image3]

