# AutoTrack-TurtleBot3

A comprehensive robotics project utilizing TurtleBot3, Python, and ROS for autonomous navigation and ball tracking.

---

## Project Overview

AutoTrack-TurtleBot3 is an advanced robotics project focused on developing autonomous navigation capabilities using TurtleBot3. The project integrates radar-based sensing, PID-controlled motion, and real-time ball detection to enable the robot to autonomously detect and navigate toward a ball.

---

## Project Structure

- **ball_detection**: Contains scripts and models for real-time ball detection.
- **robot_controller**: Houses the control algorithms, including PID controllers for motion.
- **turtlebot3_simulations**: Includes simulation files for testing and refining the system in Gazebo.

---

## Technologies Used

- Python: Primary programming language for scripting and algorithm development.
- ROS (Robot Operating System): Middleware for communication between robot components.
- Gazebo: Simulation environment for testing and validation.
- OpenCV: Library for image processing and object detection.
- PID Control: Algorithm for precise motion control.

---

## Setup Instructions

```bash
pip install opencv-python numpy

git clone https://github.com/benxta4424/AutoTrack-TurtleBot3.git

cd AutoTrack-TurtleBot3

