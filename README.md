## Kidnapped Vehicle
[![C++](https://img.shields.io/badge/C++-Solutions-blue.svg?style=flat&logo=c%2B%2B)](http://www.cplusplus.org/)
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

In this project we utilize a particle filter approach to achieve a high accuracy for the localization module on a self driving car. This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

The localization module is a critical one in a self driving vehicle. Localization can be defined as predicting the location of a vehicle with high accuracy in the range 3-10 cm, one way to localize a vehicle is by using data from the Global Positioning System (GPS), which makes use of triangulation to predict the position of an object detected by multiple satellites. But GPS doesn't always provide high accuracy data, e.g. In case of strong GPS signal, the accuracy in location is in the range of 1-3 m, whereas for a weak GPS signal, the accuracy drops to a range of 10-50 m. Hence the use of only GPS is not reliable and desirable.

To achieve an accuracy of 3-10 cm, sensor information from Laser (LIDAR) and/or Radial distance and angle sensor (RADAR) is used and fused together using a Particle Filter.

<p align="center"> 
<img src="https://github.com/ajimenezjulio/P6_Kidnapped-Vehicle/blob/master/docs/kidnapped.gif">
</p>


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
Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d).

## The Map
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns.
1. x position
2. y position
3. landmark id

## Goal
In this project, a vehicle is kidnapped inside a closed environment and has no idea of its location. This environment is simulated in [Udacity's self driving car simulator](https://github.com/udacity/self-driving-car-sim/releases). The vehicle travels through the environment and takes roughly 2400 steps with change in orientation and position. The goal is to predict the location of vehicle using a Particle Filter approach implemented in C++. The error between ground truth location of robot and the predicted location should be minimal. Additionally, the program must be performant enough to run within 100 seconds while maintaining minimal error.

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
    ├── map_data.txt   
├── src
    ├──  helper_functions.h
    ├──  main.cpp
    ├──  map.h
    ├──  particle_filter.cpp
    ├──  particle_filter.h
```

The localization module was implemented as described below:

1. A noisy measurement from GPS sensor was received and used to initialize the position of vehicle. This measurement included the x coordinate, y coordinate (both in m) and the theta (orientation) of vehicle in radian. Noise is modelled by Gaussian distribution for all the previously described parameters. The number of particles chosen for this project was 30 and were initialized to locations taken from normal distribution with mean equal to the location received from GPS and standard deviation equal to the GPS measurement uncertainty.

2. Global map of environment is initialized. This map is represented by a list x and y coordinates of landmarks in the environment.
  
3. Once map and particles are initialized, the vehicle implements the **Prediction** step in which the location of each particle at next time step is predicted. This is done by using information of control inputs and time elapsed between time steps. The control inputs are nothing but magnitude of velocity (v) and yaw rate (θ). Location update is done with the help of the formulas given below:


<p align="center" style="text-align: center;"><img align="center" src="https://i.upmath.me/svg/%0A%5Cbegin%7Btabular%7D%7Bc%7Cc%7D%0A%5Cwidehat%5Ctheta%20%3D%200%20%26%20%5Cwidehat%5Ctheta%20%5Cneq%200%20%5C%5C%0A%5Chline%0A%5Cnoalign%7B%5Cvskip%202mm%7D%0A%5Cbegin%7Baligned%7D%5Bt%5D%20%25%20placement%3A%20default%20is%20%22center%22%2C%20options%20are%20%22top%22%20and%20%22bottom%22%0Ax_f%20%26%3D%20x_0%20%2B%20v(dt)%20(%5Ccos%7B%5Ctheta%7D_0)%20%5C%5C%0Ay_f%20%26%3D%20y_0%20%2B%20v(dt)%20(%5Csin%7B%5Ctheta%7D_0)%20%5C%5C%20%0A%5Ctheta_f%20%26%3D%20%5Ctheta_0%0A%5Cend%7Baligned%7D%20%0A%26%0A%5Cbegin%7Baligned%7D%5Bt%5D%0Ax_f%20%26%3D%20x_0%20%2B%20%5Cdfrac%7Bv%7D%7B%5Chat%5Ctheta%7D%5B%5Csin(%5Ctheta_0%20%2B%20%5Chat%5Ctheta(dt))%20-%20%5Csin(%5Ctheta_0)%5D%20%5C%5C%0Ay_f%20%26%3D%20y_0%20%2B%20%5Cdfrac%7Bv%7D%7B%5Chat%5Ctheta%7D%5B%5Ccos(%5Ctheta_0)%20-%20%5Ccos(%5Ctheta_0%20%2B%20%5Chat%5Ctheta(dt))%5D%20%5C%5C%0A%5Ctheta_f%20%26%3D%20%5Ctheta_0%20%2B%20%5Chat%5Ctheta(dt)%0A%5Cend%7Baligned%7D%20%0A%5Cend%7Btabular%7D%0A" alt="
\begin{tabular}{c|c}
\widehat\theta = 0 &amp; \widehat\theta \neq 0 \\
\hline
\noalign{\vskip 2mm}
\begin{aligned}[t] % placement: default is &quot;center&quot;, options are &quot;top&quot; and &quot;bottom&quot;
x_f &amp;= x_0 + v(dt) (\cos{\theta}_0) \\
y_f &amp;= y_0 + v(dt) (\sin{\theta}_0) \\ 
\theta_f &amp;= \theta_0
\end{aligned} 
&amp;
\begin{aligned}[t]
x_f &amp;= x_0 + \dfrac{v}{\hat\theta}[\sin(\theta_0 + \hat\theta(dt)) - \sin(\theta_0)] \\
y_f &amp;= y_0 + \dfrac{v}{\hat\theta}[\cos(\theta_0) - \cos(\theta_0 + \hat\theta(dt))] \\
\theta_f &amp;= \theta_0 + \hat\theta(dt)
\end{aligned} 
\end{tabular}
" /></p>

4. After prediction step, the vehicle implements the **Update** step by using the information provided by the sensors (LIDAR and RADAR). However the coordinates of these measurements are relative to our vehicle and not the map, so a homogenous transformation is performed to map the vehicle coordinates to the global map ones in the following way.

<p align="center" style="text-align: center;"><img align="center" src="https://i.upmath.me/svg/%0A%5Cbegin%7Bbmatrix%7D%0Ax_%7Bmap%7D%20%5C%5C%0Ay_%7Bmap%7D%20%5C%5C%0A1%0A%5Cend%7Bbmatrix%7D%0A%26%20%3D%0A%5Cbegin%7Bbmatrix%7D%0A%5Ccos%5Ctheta%20%26%20-%5Csin%5Ctheta%20%26%20x_%7Bparticle%7D%20%5C%5C%0A%5Csin%5Ctheta%20%26%20%5Ccos%5Ctheta%20%26%20y_%7Bparticle%7D%5C%5C%0A0%20%26%200%20%26%201%0A%5Cend%7Bbmatrix%7D%0A%5Ctimes%0A%5Cbegin%7Bbmatrix%7D%0Ax_%7Bvehicle%7D%20%5C%5C%0Ay_%7Bvehicle%7D%5C%5C%0A1%0A%5Cend%7Bbmatrix%7D%20%0A%5C%5C%20%0A%5C%5C%0Ax_%7Bmap%7D%20%3D%20x_%7Bparticle%7D%20%2B%20(%5Ccos%5Ctheta%20%5Ctimes%20x_%7Bvehicle%7D)%20-%20(%5Csin%5Ctheta%20%5Ctimes%20y_%7Bvehicle%7D)%20%5C%5C%0Ay_%7Bmap%7D%20%3D%20y_%7Bparticle%7D%20%2B%20(%5Csin%5Ctheta%20%5Ctimes%20x_%7Bvehicle%7D)%20%2B%20(%5Ccos%5Ctheta%20%5Ctimes%20y_%7Bvehicle%7D)%0A" alt="
\begin{bmatrix}
x_{map} \\
y_{map} \\
1
\end{bmatrix}
&amp; =
\begin{bmatrix}
\cos\theta &amp; -\sin\theta &amp; x_{particle} \\
\sin\theta &amp; \cos\theta &amp; y_{particle}\\
0 &amp; 0 &amp; 1
\end{bmatrix}
\times
\begin{bmatrix}
x_{vehicle} \\
y_{vehicle}\\
1
\end{bmatrix} 
\\ 
\\
x_{map} = x_{particle} + (\cos\theta \times x_{vehicle}) - (\sin\theta \times y_{vehicle}) \\
y_{map} = y_{particle} + (\sin\theta \times x_{vehicle}) + (\cos\theta \times y_{vehicle})
" /></p>

Not all the particles can be in the vehicle's range at a given momment, so we retain only those that are in the range of the sensors from the particle position, and after this filtering process we map every sensor observation to a landmark using 1-NN (nearest neightbour algorithm). Finally the weight of the particle is updated using a Multivariate Gaussian Distribution for all observations associated to landmarks using the formula below (individual probabilities).


<p align="center" style="text-align: center;"><img align="center" src="https://i.upmath.me/svg/%0AP(x%2Cy)%20%3D%20%5Cdfrac%7B1%7D%7B2%5Cpi%5Csigma_x%5Csigma_y%7D%20%5CBigg%7Be%7D%5E%7B-%5Cleft(%5Cdfrac%7B(x-%5Cmu_x)%5E2%7D%7B2%5Csigma_x%5E2%7D%20%2B%20%5Cdfrac%7B(y-%5Cmu_y)%5E2%7D%7B2%5Csigma_y%5E2%7D%5Cright)%7D%0A" alt="
P(x,y) = \dfrac{1}{2\pi\sigma_x\sigma_y} \Bigg{e}^{-\left(\dfrac{(x-\mu_x)^2}{2\sigma_x^2} + \dfrac{(y-\mu_y)^2}{2\sigma_y^2}\right)}
" /></p>

5. The last step is resampling the particles with replacement using their normalized weight as probabilities of appearance causing removal of the unlikely ones and more presence of the likely ones.
