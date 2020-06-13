## Extended Kalman Filter
[![Python 3.6](https://img.shields.io/badge/python-3.6-blue.svg)](https://www.python.org/downloads/release/python-360/)
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

In this project we utilize a particle filter approach to achieve a high accuracy for the localization module on a self driving car. This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

The localization module is a critical one in a self driving vehicle. Localization can be defined as predicting the location of vehicle with high accuracy in the range 3-10 cm, one way to localize a vehicle is by using data from the Global Positioning System (GPS), which makes use of triangulation to predict the position of an object detected by multiple satellites. But GPS doesn't always provide high accuracy data, e.g. In case of strong GPS signal, the accuracy in location is in the range of 1-3 m, whereas for a weak GPS signal, the accuracy drops to a range of 10-50 m. Hence the use of only GPS is not reliable and desirable.

To achieve an accuracy of 3-10 cm, sensor information from Laser sensor (LIDAR) and/or Radial distance and angle sensor (RADAR) is used and fused together using a Particle Filter.

## Dependencies

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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)

## Running the Code
Some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:
```
> ./clean.sh
> ./build.sh
> ./run.sh
```
Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

## Implementation
The directory structure of this repository is as follows:

```
.
├── build.sh
├── clean.sh
├── CMakeLists.txt
├── README.md
├── run.sh
├── data
├   ├── map_data.txt   
├
├── src
    ├──  helper_functions.h
    ├──  main.cpp
    ├──  map.h
    ├──  particle_filter.cpp
    ├──  particle_filter.h
```

