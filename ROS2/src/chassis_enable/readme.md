# 🚗 chassis_enable - ROS 2 Package

The `chassis_enable` package provides utility nodes to control and manage chassis motion for a robot operating under ROS 2. It is designed to work alongside the Nav2 navigation stack and SmartCar's control interface.

---

## 📦 Package Overview

This package contains **three nodes**:

### 1. `cmd_vel_relay_node.py`

A node that:
- Subscribes to: `/cmd_vel`
- Publishes to: `/cmd_vel_out`
- **Behavior**:
  - Publishes velocity commands at a constant rate of **100 Hz**.
  - If no new `/cmd_vel` messages are received, it publishes **zero velocity**.
  - Ensures smooth and continuous control signal output, even when no input is received.

---

### 2. `chassis_enable_client`

A simple node that:
- Calls the `enable` service exposed by the **SmartCar** node.
- Used to **enable the chassis** before motion is allowed.
- Typically run once at system startup or before initiating teleop/navigation.

---

### 3. `assisted_teleop_node`

This node:
- Enables **manual teleoperation** while still allowing Nav2 to retain control.
- Used for scenarios where human assistance is required during autonomous navigation (e.g., tight corners, docking).
- Can be integrated with a joystick, keyboard teleop, or other control interfaces.

---

## 🔧 Usage

### Launch Example

You can create your own launch file to start the nodes or run them individually:

```bash
ros2 run chassis_enable cmd_vel_relay_node.py
ros2 run chassis_enable chassis_enable_client
ros2 run chassis_enable assisted_teleop_node

