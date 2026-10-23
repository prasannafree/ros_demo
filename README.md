# ROS 2 Publisher and Subscriber Demo

This repository contains a Python ROS 2 package (`pub_sub`) with:

### Wheel Speed Nodes
- `left_wheel_pub`: publishes left wheel rotational speed (`std_msgs/msg/Float64`) to `/left_wheel_speed`
- `right_wheel_pub`: publishes right wheel rotational speed (`std_msgs/msg/Float64`) to `/right_wheel_speed`
- `wheel_speed_sub`: subscribes to both `/left_wheel_speed` and `/right_wheel_speed` and prints the live speeds

### Velocity & Odometry Nodes
- `odom_pub`: publishes a constant velocity as `geometry_msgs/msg/Twist`
- `vel_sub`: subscribes to velocity and estimates position over time

## Workspace layout

```text
ros2_ws/
└── src/
    └── pub_sub/
```

Generated ROS 2 directories (`build/`, `install/`, and `log/`) are intentionally ignored by Git.

## Build

From the repository root, source your ROS 2 installation and run:

```bash
cd ros2_ws
colcon build
source install/setup.bash
```

## Run Wheel Speed Nodes

In separate terminals, source the workspace and run:

```bash
# Terminal 1: Left Wheel Publisher
ros2 run pub_sub left_wheel_pub

# Terminal 2: Right Wheel Publisher
ros2 run pub_sub right_wheel_pub

# Terminal 3: Combined Subscriber
ros2 run pub_sub wheel_speed_sub
```

## Visualize with RQT

Launch `rqt_graph` to inspect node and topic connections:

```bash
ros2 run rqt_graph rqt_graph
```

