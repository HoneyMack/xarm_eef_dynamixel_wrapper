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

# How to Run
The communication method with the Dynamixels connected to eef is almost the same as [DynamixelSDK](https://github.com/ROBOTIS-GIT/DynamixelSDK/tree/main/python).  
The difference is that instead of the default `PortHandler`, you should use `EndEffectorPortHandler` ( Please see src/endeffector_port_handler.py for detail).  
The argument for this is `xArmAPI`, and its instantiation follows the same method as described in the [xArm-Python-SDK](https://github.com/xArm-Developer/xArm-Python-SDK).

## Run Sample Code
```bash
cd xarm_eef_dynamixel_wrapper
python sample/read_write.py
```
