# EchoBot

EchoBot is an open-source robotic ultrasound system that integrates core libraries for building generic and autonomous robotic US applications.

### Features

Several of the essential components are implemented in the frameworks FAST and libromocc. This include

* **Sensor interface** - Ultrasound scanners, robots and depth cameras
* **Machine Learning** - Real-time inference through several supported engines, such as TensorFlow.
* **Visualization** - Supports 3D and 2D rendering
* **GUI Widgets** - Premade widgets that can be used in multiple applications

### Structure

EchoBot is written in C++ using CMake, Qt, Eigen, libromocc and FAST.


### Development instructions

#### Option 1. Install with FAST preinstalled on your system

Build and install the echobot branch of [FAST](https://github.com/androst/FAST/tree/echobot) on your system. Remember to enable building
Tensorflow, Realsense and OpenIGTLink in the CMake. Optionally enable the Clarius module. After installation, you can build EchoBot on 
your system by pointing to the FAST install directory. 

Linux (Ubuntu 18.04):
```bash
git clone https://github.com/androst/EchoBot.git
cd EchoBot
mkdir build
cd build
cmake .. -DFAST_DIR=/path/to/FAST/cmake/
make -j8
```

#### Option 2. Build FAST with EchoBot

This is not working as intended at the moment. 
