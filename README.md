# ROS 2 Publisher and Subscriber Demo

This repository contains a Python ROS 2 package with:

- `odom_pub`: publishes a constant velocity as `geometry_msgs/msg/Twist`
- `vel_sub`: subscribes to that velocity and estimates position over time

## Workspace layout

```text
ros2_ws/
└── src/
    └── pub_sub/
```

Generated ROS 2 directories (`build/`, `install/`, and `log/`) are intentionally
ignored by Git.

## Build

From the repository root, source your ROS 2 installation and run:

```bash
cd ros2_ws
colcon build
source install/setup.bash
```

## Run

In separate terminals, source the workspace and run:

```bash
ros2 run pub_sub odom_pub
ros2 run pub_sub vel_sub
```
