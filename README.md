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
- **RGB Filtering**:
  - Extracts **green/yellow** pixels corresponding to a tennis ball.
  - Uses predefined color thresholds to differentiate the ball from background objects.
- **Point Cloud Processing**:
  - Converts **PointCloud2** data from the camera into a structured array.
  - Extracts the 3D coordinates of detected pixels that match the ball's color signature.
- **Frame Transformation**:
  - The ball’s coordinates are detected in the **camera frame**.
  - The system retrieves the **TF transformation chain** from the **camera frame** to the **Odometry frame (odom)**.
  - Applies **homogeneous transformations** to convert the ball’s detected position into the **odom frame**, allowing global positioning.
- **Stabilization Check**:
  - If the ball position has **low variance over multiple frames**, it's marked as stationary.
  - This prevents chasing moving objects falsely identified as balls.

### 2. **Navigation to Ball** (Using Odometry & TF)
- **Compute Goal Position**:
  - The transformed **(x, y)** coordinates of the ball in **odom** become the robot’s new goal.
  - Publishes a **visual marker** for debugging.
- **Control System**:
  - Uses **linear velocity (proportional to distance)** to approach the ball.
  - Adjusts **angular velocity** to minimize heading error.
  - Uses **obstacle detection** from LiDAR to slow down or avoid collisions.
- **Obstacle Avoidance**:
  - If LiDAR detects an obstacle within **safe distance**, the robot adjusts its path.
  - If the obstacle is too close, the robot stops until it clears.

### 3. **Picking Up the Ball**
- Uses the **KidnapStatus** message to detect when the ball is picked up.
- Ensures a proper pickup before transitioning to human search mode.

### 4. **Human Detection & Return**
- Uses **blue color filtering** to identify the human.
- Converts detected human position into **goal coordinates**.
- Uses **LiDAR and Odometry** for safe approach.
- If the detected human position **oscillates too much**, it waits for stabilization before proceeding.

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



