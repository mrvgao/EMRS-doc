Robot System Overview
======================

The **Patrolling Trash-Collecting Robot** integrates multiple hardware and software components to achieve autonomous environmental maintenance. This document provides an overview of the system architecture, its key subsystems, and the integration strategies employed.


System Architecture
--------------------

The system comprises several interconnected subsystems:

- **Mobile Robot Base**: Provides mobility, supports payload, and houses primary sensors.
- **Robotic Arm**: Handles manipulation tasks including trash picking and placement.
- **Sensors**: Includes LiDAR for navigation, RGBD camera for object detection and tracking.
- **Computational Unit**: Processes sensor data, executes software modules for navigation, detection, and manipulation.


Subsystem Integration
----------------------

Subsystem integration is achieved through ROS2 middleware, leveraging a modular approach:

- **Navigation and Localization**: Integrated through ROS2 Nav2 stack and RTAB-Map for robust localization and environment mapping.
- **Perception**: YOLO-based object detection combined with point cloud segmentation and tracking using TF frames.
- **Manipulation**: Managed via MoveIt2, enabling collision-free arm movement and precise grasping and placement of objects.


Data Flow
----------

The data flow in the robot system:

1. **Sensor Data Acquisition**: LiDAR and RGBD camera provide continuous streams of environmental data.
2. **Perception and Detection**: YOLO detects objects; Lidar and RGBD camera data is combined to establish accurate object positions.
3. **Localization and Navigation**: Sensor data is processed by RTAB-Map for 3D mapping and localization. Nav2 plans and executes navigation paths.
4. **Manipulation Commands**: TF-based pose estimation and MoveIt2 determine precise arm positions and grasping sequences.

This robust architecture ensures reliable autonomous operation in dynamic environments.
