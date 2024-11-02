# Swift Pico Drone Navigation and Inventory Validation in Gazebo Simulation

## Overview
This project involves autonomous navigation and inventory validation using the Swift Pico Drone in a simulated warehouse environment built with Gazebo. The drone utilizes ROS2 for real-time communication and control, enabling precise movement, collision avoidance, and package identification. This work extends across multiple tasks involving 2D mapping, path planning, and validation of packages at known locations.

## Features
* **2D Mapping and Obstacle Detection**: Utilizes Aruco markers to generate a 2D bitmap of the warehouse for path planning and obstacle identification.
* **Autonomous Navigation**: Implements a custom ROS2 action server for waypoints navigation with real-time error checking, ensuring precise movement within ±0.6 meters.
* **Inventory Validation**: Conducts package identification at specific coordinates, with stable hovering for 3 seconds at each package location.
* **Service & Action Communication**: Utilizes custom ROS2 services and action clients/servers for managing waypoint and path planning requests with consistent feedback.

## Project Structure
```
swift_pico_project/
├── src/
│   ├── warehouse_service.py    # Service for path planning and obstacle data
│   ├── pico_server_2c.py      # Action server to handle waypoint navigation
│   ├── pico_client_2c.py      # Action client to send waypoints and receive feedback
│   ├── path_planning_service.py# Path planning for warehouse navigation
│   └── waypoint_service.py     # Manages waypoints for the drone
├── models/
│   └── swift_pico_description/ # 3D models and URDF files for drone simulation
├── launch/
│   └── task_2c.launch.py      # Launch file for Task 2C in Gazebo
└── README.md
```

## Prerequisites
* ROS2 (Galactic or newer)
* Python 3.8+
* Gazebo
* OpenCV (for Aruco marker detection)
* WhyCon for height stabilization

## Setup
1. **Clone the repository**
```bash
git clone https://github.com/K-Jyotiraditya/swift_pico_project.git
cd swift_pico_project
```

2. **Install Dependencies**
Install ROS2 packages and OpenCV using:
```bash
sudo apt install ros-<distro>-gazebo-ros-pkgs python3-opencv
```

3. **Set Up the Environment**
Ensure the Gazebo models are accessible:
```bash
export GZ_SIM_RESOURCE_PATH="$HOME/swift_pico_project/models"
```

4. **Build the Workspace**
```bash
cd swift_pico_project
colcon build
source install/setup.bash
```

## Usage
1. **Launch the Simulation**
```bash
ros2 launch swift_pico task_2c.launch.py
```

2. **Run the ROS2 Nodes**
In separate terminals, start the service, server, and client nodes:
```bash
# Terminal 1: Warehouse Service
ros2 run swift_pico warehouse_service.py

# Terminal 2: Action Server
ros2 run swift_pico pico_server_2c.py

# Terminal 3: Action Client
ros2 run swift_pico pico_client_2c.py
```

## Task Workflow
* **Task 2A**: Waypoint navigation in Gazebo simulation.
* **Task 2B**: Path planning using Aruco-based 2D mapping.
* **Task 2C**: Inventory validation with package detection and hover at predefined points.

## Technical Highlights
* **ROS2 Communication**: Custom service and action interfaces for waypoint retrieval, path planning, and navigation feedback.
* **Gazebo Integration**: Simulates warehouse environment and Swift Pico model for realistic testing.
* **Collision and Error Checking**: Monitors potential collisions and height violations throughout navigation.

## Results
The drone successfully navigates the simulated warehouse, hovers at each package location for 3 seconds, and maintains stable flight within the defined error margins. The final implementation allows for dynamic waypoint adjustments and efficient route planning.



## License
This project is licensed under the MIT License.
