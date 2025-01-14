# Delivery Robot Project

This project implements a ROS2-based delivery robot system with navigation, delivery management, and visualization capabilities.

## Project Structure

The project consists of three main packages:

### 1. Navigation Package

- **node_navigation.py**: Manages robot position and movement
- **node_obstacle_avoidance.py**: Handles obstacle detection and avoidance

### 2. Delivery Manager Package

- **node_service_manager.py**: Manages delivery services
- **node_action_manager.py**: Handles delivery status and progress

### 3. Visualization Package

- **node_rviz_visualization.py**: Provides RViz visualization
- **node_log_display.py**: Displays system logs

## Installation

1. Create a ROS2 workspace:

```bash
mkdir -p ~/delivery_ws/src
cd ~/delivery_ws/src
```

2. Clone the repository:

```bash
git clone <repository_url>
```

3. Build the packages:

```bash
cd ~/delivery_ws
colcon build
```

4. Source the workspace:

```bash
source ~/delivery_ws/install/setup.bash
```

## Running the Project

1. Launch the navigation nodes:

```bash
ros2 run navigation node_navigation
ros2 run navigation node_obstacle_avoidance
```

2. Launch the delivery manager:

```bash
ros2 run delivery_manager node_service_manager
ros2 run delivery_manager node_action_manager
```

3. Launch the visualization:

```bash
ros2 run visualization node_rviz_visualization
ros2 run visualization node_log_display
```

4. Start RViz:

```bash
rviz2
```

## Testing

To test the system:

1. Start a delivery:

```bash
ros2 service call /start_delivery custom_interfaces/srv/StartDelivery "{destination: 'point_a'}"
```

2. Monitor delivery status:

```bash
ros2 topic echo /delivery_status
```

3. End a delivery:

```bash
ros2 service call /end_delivery std_srvs/srv/Trigger
```

## Dependencies

- ROS2 (tested with Humble)
- Python 3.8+
- Required ROS2 packages:
  - geometry_msgs
  - nav_msgs
  - sensor_msgs
  - visualization_msgs
  - std_srvs

## License

This project is licensed under the Apache License 2.0.
