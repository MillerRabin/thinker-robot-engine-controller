# Thinker Robot Engine Control

Engine control code for every manipulator part. Every part contain CPU RP2040, IMU BNO085, CAN BUS, and 1-4 engines depending on functionality.

Parts
* Shoulder. RP2040, 2 servo engines Y and Z. IMU BNO085.
* Elbow. RP2040, 1 servo engine Y. IMU BNO085.
* Wrist. RP2040, 2 servo engines Y and Z. IMU BNO085.
* Claw. RP2040 3 servo engines X, Y and Gripper. IMU Witmotion WT901B. Two laser range detectors


FreeRTOS is used.
All parts saving their IMU Callibration in flash memory. 


## Usage


1. Configure the build process: `cmake -S . -B build/`.
1. Build the app: `cmake --build build`.
1. use `./deploy-shouler.sh` to deploy shoulder part

## Debug vs Release
```shell
cmake -S . -B build -D CMAKE_BUILD_TYPE=Debug
```

For a release build, which among various optimisations omits UART debugging code, call:

```shell
cmake -S . -B build -D CMAKE_BUILD_TYPE=Release
```

Follow both of these commands with the usual

```shell
cmake --build build
```