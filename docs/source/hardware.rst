Hardware Description and Specifications
========================================

The **Patrolling Trash-Collecting Robot** utilizes carefully selected hardware components that enable autonomous navigation, effective trash detection, and precise manipulation.


Mobile Robot Base
------------------

The mobile robot provides the foundational platform for movement, sensor mounting, and payload support. Possible candidate robots include:

- **Limo Cobot**: Integrated robotic arm, LiDAR, RGBD camera; ROS Foxy compatibility.
- **ROSbot XL (Manipulator Pro)**: Full ROS2 support, high payload capacity, onboard sensors.
- **Limo ROS2**: ROS2 Humble compatible base, requires external arm integration (e.g., OpenManipulator X).


Robotic Arm
------------

A robotic arm capable of precise manipulation tasks is integrated:

- **Compatibility**: MoveIt2 compliant for effective path planning and collision avoidance.
- **Potential Arm Models**: OpenManipulator X, Limo integrated arm, or similar robotic manipulators with sufficient reach and payload capacity.


Sensors
--------

Critical sensors employed by the robot include:

- **LiDAR Sensor**: Provides high-resolution data for navigation, obstacle avoidance, and mapping.
- **RGBD Camera**: Captures visual and depth data used for real-time object detection, tracking, segmentation, and pose estimation.


Computational Hardware
-----------------------

The onboard computational hardware handles processing requirements:

- **Embedded Computing Unit**: GPU-accelerated device (e.g., NVIDIA Jetson series) to efficiently handle YOLO inference, RTAB-Map processing, and MoveIt2 trajectory planning.
- **Power Management**: Reliable power supply system with sufficient capacity for sustained autonomous operation.


Integration
------------

The hardware components are integrated into a cohesive system, leveraging ROS2 middleware to:

- Facilitate efficient communication between subsystems.
- Enable seamless data acquisition, processing, and actuation tasks.


Specifications Summary
-----------------------

+-------------------+----------------------------------------------+
| Component         | Specification Details                        |
+===================+==============================================+
| Mobile Base       | Moderate payload and navigation              |
+-------------------+----------------------------------------------+
| Robotic Arm       | High precision, adequate reach               |
+-------------------+----------------------------------------------+
| Lidar             | Accurate distance measurement                |
+-------------------+----------------------------------------------+
| RGBD Camera       | High resolution, real-time depth and RGB     |
+-------------------+----------------------------------------------+
| Computing Unit    | GPU acceleration, robust processing          |
+-------------------+----------------------------------------------+
| Power Supply      | Sufficient for continuous operation          |
+-------------------+----------------------------------------------+


The chosen hardware ensures robust and reliable operation, supporting the robot’s autonomous environmental cleaning capabilities effectively.