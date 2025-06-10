## Modified Package to be Compatible with Humble
These is the ROS1 and ROS2 packages for Segway chassis 100, 200, and 400 series.

## ROS2 hints
1. Remove the ROS folder.
2. Build the package.
3. Add the external library to the environment. According to these messages from the compiler
```bash
-- Segway Robotics Notice: The external library and include path can be modified by users.
-- Segway Robotics Notice: The external include path is /home/user/ros2_ws/src/RMP220-SDK/ROS2/src/segwayrmp/../../../LibAPI/include
-- Segway Robotics Notice: The external library path is /home/user/ros2_ws/src/RMP220-SDK/ROS2/src/segwayrmp/../../../LibAPI/lib
-- Segway Robotics Notice: This system is running on an x86_64 CPU.
-- Segway Robotics Notice: The name of the external library is /home/user/ros2_ws/src/RMP220-SDK/ROS2/src/segwayrmp/../../../LibAPI/lib/libctrl_x86_64.so
```

To do so 
```bash
export LD_LIBRARY_PATH=/ssd/workspaces/ros2_ws/src/RMP220-SDK/LibAPI/lib:$LD_LIBRARY_PATH
```


<p align="center">
  <img src="./Pictures/segway-robotics_logo.png" width="25%" />
</p>

# RMP220-SDK

Segway-Ninebot company gives developers the SDK for the RMP220 product.

For further usage instructions, please refer to the [Wiki](https://github.com/SegwayRoboticsSamples/RMP220-SDK/wiki).

