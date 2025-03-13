# About
This README is English version. Japanese version is [here](README_jp.md).

This is a wrapper for controlling the Dynamixel servo connected to the end-effector of the xArm/ Lite6 robot.



# Installation Guide
## Clone the Repository
```bash
git clone https://github.com/matsuolab/xarm_eef_dynamixel_wrapper
```

## Install Required Packages
```bash
cd xarm_eef_dynamixel_wrapper
pip install .
```

# How to Use
This wrapper provides a high-level API and a low-level API.
## High-level API
You can control the Dynamixel servo connected to eef using the high-level API `EEFAPI`.
Please see a [example](examples/use_highlevel_api.py) to know how to use this api.

## Low-level API
You can finely control the Dynamixel servo connected to eef using the low-level API `EndEffectorPortHandler` and `DynamixelSDK`.
The communication method with the Dynamixels connected to eef is almost the same as [DynamixelSDK](https://github.com/ROBOTIS-GIT/DynamixelSDK/tree/main/python).  
The difference is that instead of the default `PortHandler`, you should use `EndEffectorPortHandler` ( Please see src/endeffector_port_handler.py for detail).  
The argument for this is `xArmAPI`, and its instantiation follows the same method as described in the [xArm-Python-SDK](https://github.com/xArm-Developer/xArm-Python-SDK).

Please see examples like [ping](examples/ping.py), [read_write](examples/read_write.py) and etc. to know how to use this api.