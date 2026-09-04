## Modified Package to be Compatible with Humble
ROS2 packages for Segway chassis 100, 200, and 400 series.

## ROS2 hints
1. Build the package.
2. Add the external library to the environment. According to these messages from the compiler
```bash
-- Segway Robotics Notice: The external library and include path can be modified by users.
-- Segway Robotics Notice: The external include path is /home/user/ros2_ws/src/RMP220-SDK/ROS2/src/segwayrmp/../../../LibAPI/include
-- Segway Robotics Notice: The external library path is /home/user/ros2_ws/src/RMP220-SDK/ROS2/src/segwayrmp/../../../LibAPI/lib
-- Segway Robotics Notice: This system is running on an x86_64 CPU.
-- Segway Robotics Notice: The name of the external library is /home/user/ros2_ws/src/RMP220-SDK/ROS2/src/segwayrmp/../../../LibAPI/lib/libctrl_x86_64.so
```

Go to the src of the package and find the LibAPI/lib folder
If the host is x86, follow this
```bash
sudo cp libctrl_x86_64.so /usr/local/lib/ && sudo chmod 
```

If the host is ARM64, follow this
```bash
sudo cp libctrl_arm64-v8a.so /usr/local/lib/ && sudo chmod 
```


