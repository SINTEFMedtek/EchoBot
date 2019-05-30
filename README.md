# EchoBot

EchoBot is an open-source robotic ultrasound system that integrates core libraries for building generic and autonomous robotic US applications.

### Features

Several of the essential components are implemented in the frameworks FAST and libromocc. This include

* **Sensor interface** - Ultrasound scanners, robots and depth cameras
* **Machine Learning** - Real-time inference through several supported engines, such as TensorFlow.
* **Visualization** - Supports 3D and 2D rendering
* **GUI Widgets** - Premade widgets that can be used in

### Structure

EchoBot is written in C++ using CMake, Qt, Eigen, libromocc and FAST.


**Build instructions**

```bash
git clone https://github.com/androst/EchoBot.git
cd EchoBot
mkdir build
cd build
cmake ..
make -j8
```