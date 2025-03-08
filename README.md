# TTB Fetch - TurtleBot4 Fetching Robot

## Overview
TTB Fetch is a **ROS2-based** robotic system that enables a **TurtleBot4** to play fetch. The robot autonomously detects, fetches, and returns a ball to a human. The system uses **sensor fusion, computer vision, and navigation algorithms** to track the ball, avoid obstacles, and navigate back to the human for repeated fetch cycles.

## Key Features
- **Ball Detection**: Uses **RGB camera and point cloud processing** to identify the ball's location.
- **Autonomous Navigation**: Leverages **Odometry, LiDAR, and TF transformations** for localization and movement.
- **Obstacle Avoidance**: Implements a **safety distance mechanism** to avoid obstacles during movement.
- **Ball Fetching Logic**: Detects a stable ball position and navigates toward it.
- **Return to Human**: Identifies the human using **color-based detection** and returns the ball.
- **ROS2 Integration**: Communicates using **ROS2 topics, TF transformations, and KidnapStatus messages**.

## How It Works (Algorithm Overview)
1. **Search for the Ball**: 
   - The robot scans the environment using its **depth camera and LiDAR**.
   - Detects a ball by identifying **consistent color patterns** in multiple samples.
   - Ensures the ball is stationary before committing to fetch it.

2. **Navigate to the Ball**:
   - Computes the **goal position** for the robot using detected ball coordinates.
   - Uses a **PID-based control system** to move toward the ball.
   - Adjusts speed dynamically based on distance and obstacle proximity.

3. **Pick Up the Ball**:
   - Waits for the **KidnapStatus** message to confirm pickup (e.g., a person placing the ball in the robot's holder).
   - Marks the ball as fetched and switches to search mode for the human.

4. **Return to the Human**:
   - Searches for the human using **predefined color-based detection (blue markers)**.
   - Uses **Odometry and TF transformations** to navigate toward the detected human position.
   - Stops at a safe distance and waits for the human to throw the ball again.

5. **Repeat Fetch Cycle**:
   - Resets states and waits for the next ball throw.

## Detailed Breakdown
### 1. **Ball Detection** (Using Camera & Point Cloud Data)
- **RGB Filtering**: Extracts **green/yellow** pixels corresponding to a tennis ball.
- **Point Cloud Processing**:
  - Converts **PointCloud2** data to a structured array.
  - Filters and stabilizes the detected ball position over **multiple samples**.
- **Stabilization Check**:
  - If the ball position has **low variance over multiple frames**, it's marked as stationary.
  - This prevents chasing moving objects falsely identified as balls.

### 2. **Navigation to Ball** (Using Odometry & TF)
- **Compute Goal Position**:
  - Converts **camera frame** coordinates into the **Odometry frame**.
  - Publishes a **visual marker** for debugging.
- **Control System**:
  - Uses **linear velocity (proportional to distance)** to approach the ball.
  - Adjusts **angular velocity** to minimize heading error.
  - Uses **obstacle detection** from LiDAR to slow down or avoid collisions.

### 3. **Picking Up the Ball**
- Uses the **KidnapStatus** message to detect when the ball is picked up.
- Ensures a proper pickup before transitioning to human search mode.

### 4. **Human Detection & Return**
- Uses **blue color filtering** to identify the human.
- Converts detected human position into **goal coordinates**.
- Uses **LiDAR and Odometry** for safe approach.

### 5. **Obstacle Avoidance**
- Processes **LaserScan** data to determine the closest obstacle distance.
- Implements an emergency stop if an obstacle is too close.
- Uses **angular corrections** to navigate around obstacles.

## Installation & Execution
### Requirements
- ROS2 Humble or newer
- TurtleBot4
- Camera & LiDAR configured for ROS2 topics

### Running the Robot
1. **Launch ROS2**:
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 launch turtlebot4_bringup minimal.launch.py
   ```
2. **Run TTB Fetch Node**:
   ```bash
   ros2 run ttb_fetch ttb_fetch.py
   ```

## Notes
- The robot **requires a pre-configured color filter** to detect the ball and human.
- Ensure the **LiDAR, Odometry, and Camera** topics are active.
- The system assumes a **flat playing field** with no excessive obstacles.

## Credits
This project was developed by https://www.linkedin.com/in/jasonmh22345 and https://www.linkedin.com/in/kidus-fasil-59249621a 



