# Localization: Particle Filter
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

[//]: # (Image References)

[dataset_1_gif]: ./images/dataset1.gif "Localization on a dataset"

![alt text][dataset_1_gif]

Overview
---

This repository contains a C++ implementation of a 2 dimensional particle filter used to estimate the position of a car moving around a map containing the position of a known set of landmarks. Localization algorithms aim to accurately position a moving object in a map, with a very low uncertainty (e.g. less than 10cm of error) given that current GPS technologies cannot reach such high level of precision. The precision of the position estimation becomes crucial in applications such as those of self-driving car (for example to estimate the exact position within lane lines on a road).

The idea of a particle filter is that if we know the exact position in a known [map](./data/map_data.txt) of a set of landmarks (e.g. corner of a street, buildings etc.) we can compare it with various uncertain observations to estimate the location in respect to those landmarks. We can start off with an initial estimate (e.g. GPS input) and then quickly step up the precision according to subsequent sensor measurements and landmark *observations*. Even though this measurements have some uncertainty we can have an accurate estimation simply by giving a score (e.g. weight) to a set of particles distributed randomly around the map according to their distance in respect to the surrounding landmarks, the more they reflects the observations coming from the sensors the more they are likely to represent the exact location of the car. Resampling the particle set according to the weights (very) quickly kills off those particles that do no represent the position of the car. The tradeoffs are in the choice of number of particles (e.g. more particles more precision and more computation) and in the way the closest landmarks to some observation are chosen (e.g. data association).

The observations in this project come from a simulated environment thanks to the [Udacity Simulator](https://github.com/udacity/self-driving-car-sim) and are fed to the program through [WebSockets](https://en.wikipedia.org/wiki/WebSocket) messages. The [main](./src/main.cpp) file processes the incoming messages and parses the data that is then processed by the [Particle Filter](./src/particle_filter.cpp) class. 

#### Input: values provided by the simulator to the c++ program

// Sense noisy position data from the simulator

["sense_x"]

["sense_y"]

["sense_theta"]

// Previous velocity and yaw rate to predict the particle's transitioned state

["previous_velocity"]

["previous_yawrate"]

// Noisy observation (landmarks) data from the simulator, in a respective list of x/y values

["sense_observations_x"]

["sense_observations_y"]


#### Output: values provided by the c++ program to the simulator

// Best particle values used for calculating the error evaluation

["best_particle_x"]

["best_particle_y"]

["best_particle_theta"]

// Optional message data used for debugging particle's sensing and associations

// Sensed positions ID label (from landmarks) for respective (x,y)

["best_particle_associations"]

// Sensed positions

["best_particle_sense_x"] <= list of sensed x positions

["best_particle_sense_y"] <= list of sensed y positions

Particle Filter
---

The filter is [initialized](./src/particle_filter.cpp#L24) at first using simulated GPS coordinates and a set of particles are distributed randomly around this initial uncertain location using a normal distribution with the location as mean and the GPS uncertainty as standard deviation.

The second step of the filter is to [predict the position](./src/particle_filter.cpp#L47) of each particle according to the previous velocity and yaw rate. Again this is modeled using a certain amount of [noise](./src/particle_filter.cpp#L67) (e.g. account for sensor measurement uncertainty).

In the [update step](./src/particle_filter.cpp#L73) of the filter the weight of each particle (e.g. the probability for the particle to be the car) is computed using a [multivariate gaussian probability density function](https://en.wikipedia.org/wiki/Multivariate_normal_distribution#Density_function) for each landmark measurement (observation) that gives a measure of the likelihood of the measurement given our predicted state with the assumption that the sensors have gaussian noise. Given that each observation measurement is independent we can take the product of the likelihoods over all measurements. 

This steps involves several calculations:

1. To increase the efficiency first the set of map landmarks are [filtered](./src/particle_filter.cpp#L88) based on the maximum sensor range.

2. Each observation is [transformed](./src/particle_filter.cpp#L95) into map coordinates given that the measurements are expressed relative to the car position. To perform this transformation we use the position and yaw of the particle as reference and a homogenous transformation matrix:

    <a href="https://www.codecogs.com/eqnedit.php?latex=\begin{bmatrix}&space;X_{m}\\&space;Y_{m}\\&space;1&space;\end{bmatrix}&space;=&space;\begin{bmatrix}&space;cos(\theta)&space;&&space;-sin(\theta)&space;&&space;X_{p}\\&space;sin(\theta)&space;&&space;cos(\theta)&space;&&space;Y_{p}\\&space;0&space;&&space;0&space;&&space;1&space;\end{bmatrix}&space;=&space;\begin{bmatrix}&space;X_{c}\\&space;Y_{c}\\&space;1&space;\end{bmatrix}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\begin{bmatrix}&space;X_{m}\\&space;Y_{m}\\&space;1&space;\end{bmatrix}&space;=&space;\begin{bmatrix}&space;cos(\theta)&space;&&space;-sin(\theta)&space;&&space;X_{p}\\&space;sin(\theta)&space;&&space;cos(\theta)&space;&&space;Y_{p}\\&space;0&space;&&space;0&space;&&space;1&space;\end{bmatrix}&space;=&space;\begin{bmatrix}&space;X_{c}\\&space;Y_{c}\\&space;1&space;\end{bmatrix}" title="\begin{bmatrix} X_{m}\\ Y_{m}\\ 1 \end{bmatrix} = \begin{bmatrix} cos(\theta) & -sin(\theta) & X_{p}\\ sin(\theta) & cos(\theta) & Y_{p}\\ 0 & 0 & 1 \end{bmatrix} = \begin{bmatrix} X_{c}\\ Y_{c}\\ 1 \end{bmatrix}" /></a>

    Which translates into:

    <a href="https://www.codecogs.com/eqnedit.php?latex=x_{m}&space;=&space;x_{p}&space;&plus;&space;(cos(\theta)&space;\times&space;x_{c})&space;-&space;(sin(\theta)&space;\times&space;y_{c})" target="_blank"><img src="https://latex.codecogs.com/gif.latex?x_{m}&space;=&space;x_{p}&space;&plus;&space;(cos(\theta)&space;\times&space;x_{c})&space;-&space;(sin(\theta)&space;\times&space;y_{c})" title="x_{m} = x_{p} + (cos(\theta) \times x_{c}) - (sin(\theta) \times y_{c})" /></a>

    <a href="https://www.codecogs.com/eqnedit.php?latex=y_{m}&space;=&space;y_{p}&space;&plus;&space;(sin(\theta)&space;\times&space;x_{c})&space;&plus;&space;(cos(\theta)&space;\times&space;y_{c})" target="_blank"><img src="https://latex.codecogs.com/gif.latex?y_{m}&space;=&space;y_{p}&space;&plus;&space;(sin(\theta)&space;\times&space;x_{c})&space;&plus;&space;(cos(\theta)&space;\times&space;y_{c})" title="y_{m} = y_{p} + (sin(\theta) \times x_{c}) + (cos(\theta) \times y_{c})" /></a>

    Where <a href="https://www.codecogs.com/eqnedit.php?latex=x_{p}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?x_{p}" title="x_{p}" /></a>, <a href="https://www.codecogs.com/eqnedit.php?latex=y_{p}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?y_{p}" title="y_{p}" /></a> and <a href="https://www.codecogs.com/eqnedit.php?latex=\theta" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\theta" title="\theta" /></a> are the coordinates of the particle of reference.

3. Once the observations are transformed to map's coordinates we can find the [closest landmark](./src/particle_filter.cpp#L98) using a simple nearest neighbor algorithm so we can associate it to the observation.

4. Finally the [multivariate gaussian probability density](./src/particle_filter.cpp#L120) is computed and the weight of each particle is updated:

    <a href="https://www.codecogs.com/eqnedit.php?latex=P(x,y)&space;=&space;\frac{1}{2\pi\sigma_{x}\sigma_{y}}&space;\times&space;e^{-(\frac{(x-\mu_{x})^2}{2\sigma_{x}^{2}}&space;&plus;&space;\frac{(y-\mu_{y})^2}{2\sigma_{y}^{2}})}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?P(x,y)&space;=&space;\frac{1}{2\pi\sigma_{x}\sigma_{y}}&space;\times&space;e^{-(\frac{(x-\mu_{x})^2}{2\sigma_{x}^{2}}&space;&plus;&space;\frac{(y-\mu_{y})^2}{2\sigma_{y}^{2}})}" title="P(x,y) = \frac{1}{2\pi\sigma_{x}\sigma_{y}} \times e^{-(\frac{(x-\mu_{x})^2}{2\sigma_{x}^{2}} + \frac{(y-\mu_{y})^2}{2\sigma_{y}^{2}})}" /></a>

    Where x and y are the coordinates of the observation transformed to map's coordinates and <a href="https://www.codecogs.com/eqnedit.php?latex=\mu_{x}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\mu_{x}" title="\mu_{x}" /></a>, <a href="https://www.codecogs.com/eqnedit.php?latex=\mu_{y}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\mu_{y}" title="\mu_{y}" /></a> are the coordinates of the closest landmark.

The final step before starting the cycle again is the [resample](./src/particle_filter.cpp#L128) the set of particles according to the previously computed weights, randomly sampling with replacement with probability equal to the particles' weight. This has the effect to keep "alive" only those particles that have a high probability to be in the location of the car according to the observations.

Getting Started
---

In order to run the program you need the simulator provided by [Udacity](https://www.udacity.com/) which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even better [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. The version compatible with the simulator is the uWebSocketIO branch **e94b6e1**.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. ```mkdir build```
2. ```cd build```
3. ```cmake .. && make```
4. ```./particle_filter```

Note that to compile the program with debug symbols you can supply the appropriate flag to cmake: ```cmake -DCMAKE_BUILD_TYPE=Debug .. && make```.

Now the Udacity simulator can be run selecting the kidnapped vehicle project, press start and see the application in action.

![alt text][dataset_1_gif]

#### Other Dependencies

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

Environment Setup
---

This project was developed under windows using the windows subsystem for linux ([WSL](https://docs.microsoft.com/en-us/windows/wsl/install-win10)) with Ubuntu Bash 16.04 together with [Visual Studio Code](https://code.visualstudio.com/).

The steps to setup the environment under mac, linux or windows (WSL) are more or less the same:

- Review the above dependencies
- Clone the repo and run the appropriate script (./install-ubuntu.sh under WSL and linux and ./install-mac.sh under mac), this should install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) from the branch **e94b6e1**

Under windows (WSL) and linux you can make a clean installation as follows:

1. ```sudo apt-get update```
2. ```sudo apt-get install git```
3. ```sudo apt-get install cmake```
4. ```sudo apt-get install openssl```
5. ```sudo apt-get install libssl-dev```
6. ```git clone https://github.com/Az4z3l/CarND-Particle-Filter```
7. ```sudo rm /usr/lib/libuWS.so```
8. ```./install-ubuntu.sh```

#### Debugging with VS Code

Since I developed this project using WSL and Visual Studio Code it was very useful for me to setup a debugging pipeline. VS Code comes with a official Microsoft cpp extension that can be downloaded directly from the marketplace: https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools. After the installation there are a few things to setup in order to make it work with the subsystem for linux, personally I went with the default Ubuntu distribution.

For the following setup I assume that the repository was cloned in **D:/Dev/CarND-Particle-Filter/**.

##### Setup the language server (for IntelliSense)

From the official documentation [https://github.com/Microsoft/vscode-cpptools/blob/master/Documentation/LanguageServer/Windows%20Subsystem%20for%20Linux.md](https://github.com/Microsoft/vscode-cpptools/blob/master/Documentation/LanguageServer/Windows%20Subsystem%20for%20Linux.md): 

Simply Crtl+P and select "C/Cpp: Edit Configurations", this will create a c_cpp_properties.json file that can be configured as follows:

```json
{
    "name": "WSL",
    "intelliSenseMode": "clang-x64",
    "compilerPath": "/usr/bin/gcc",
    "includePath": [
        "${workspaceFolder}"
    ],
    "defines": [],
    "browse": {
        "path": [
            "${workspaceFolder}"
        ],
        "limitSymbolsToIncludedHeaders": true,
        "databaseFilename": ""
    },
    "cStandard": "c11",
    "cppStandard": "c++17"
}
```

##### Setup the Debugger

From the official documentation [https://github.com/Microsoft/vscode-cpptools/blob/master/Documentation/Debugger/gdb/Windows%20Subsystem%20for%20Linux.md](https://github.com/Microsoft/vscode-cpptools/blob/master/Documentation/Debugger/gdb/Windows%20Subsystem%20for%20Linux.md):

First install gdb in the WSL:

```
sudo apt install gdb
```

Then simply create a lunch configuration from VS Code: "Debug" -> "Add Configuration.." and setup the launch.json as follows:

```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "C++ Launch",
            "type": "cppdbg",
            "request": "launch",
            "program": "/mnt/d/Dev/CarND-Particle-Filter/build/particle_filter",
            "args": ["-fThreading"],
            "stopAtEntry": false,
            "cwd": "/mnt/d/Dev/CarND-Particle-Filter/build/",
            "environment": [],
            "externalConsole": true,
            "windows": {
                "MIMode": "gdb",
                "setupCommands": [
                    {
                        "description": "Enable pretty-printing for gdb",
                        "text": "-enable-pretty-printing",
                        "ignoreFailures": true
                    }
                ]
            },
            "pipeTransport": {
                "pipeCwd": "",
                "pipeProgram": "c:\\windows\\sysnative\\bash.exe",
                "pipeArgs": ["-c"],
                "debuggerPath": "/usr/bin/gdb"
            },
            "sourceFileMap": {
                "/mnt/d": "d:\\"
            }
        }
    ]
}
```

Note how the program is mapped directly into the file system of the WSL and piped through bash.exe (the paths are relative to the WSL environment).

Now you are ready to debug the application directly from VS Code, simply compile the application from within the WSL with the debug symbols:

```cmake -DCMAKE_BUILD_TYPE=Debug .. && make```

And run the debugger from VS Code (e.g. F5) :)
