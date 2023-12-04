# Line Following Robot using ROS2 and Raspberry Pi

## Project Overview
The Line Following Robot project is a state-of-the-art integration of robotics and software engineering, aimed at creating an autonomous robot that excels at following a predefined path. Leveraging the capabilities of ROS2 for communication and a Raspberry Pi for processing, this robot showcases precision and adaptability in its path-following tasks.

## Hardware Components

- **Raspberry Pi 4b**: Central processing unit running ROS2 nodes, processing visual data for line detection.
- **Camera Module**: Captures images for real-time track navigation.
- **Motors**: Provide mobility and steering for the robot.
- **L298N Motor Driver**: Controls motor direction and speed.
- **Power Supply**: 12V battery as the primary energy source.
- **Buck Converter**: Converts 12V to 5V for Raspberry Pi and other components.
- **Safety Fuse and Power Switch**: Ensure safe operation and protect against overloads.

## Software Architecture

- **Line Detection Node (`image_publisher.py`)**: Processes camera input to detect the line and issue navigation commands.
- **Motion Control Node (`line_follow_drive.py`)**: Translates navigation commands into motor movements to follow the line.
- **Simulation Node (`line_following_sim_node.py`)**: Facilitates simulation-based development and testing.
- **Real-World Node (`line_following_real_node.py`)**: Manages real-world operation.

## Simulation Environment

The robot's performance is rigorously tested in a simulated Gazebo environment before deployment. This ensures safe development and algorithm refinement.

## Author
Aryan Kumar Nadipally

## License
This project is licensed under the MIT License - see the LICENSE.md file for details.
