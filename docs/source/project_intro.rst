Overview of Patrolling Trash-Collecting Robot
=================================

The **Patrolling Trash-Collecting Robot** project brings together a mobile robotic platform, a manipulator arm, and intelligent perception and navigation systems to autonomously patrol an environment and collect small trash objects. This system is built on ROS2 and supports both simulation in Ignition Gazebo and deployment on a real TurtleBot4-based hardware platform.

.. contents:: Table of Contents
   :depth: 2
   :local:

Overview
--------

This project explores robotic autonomy through integrated subsystems for navigation, perception, and manipulation. The robot autonomously navigates a mapped space, detects trash using computer vision, approaches the object, and uses a robotic arm to pick and store it.

The full system has been implemented both in simulation and on a real robot, offering a complete pipeline from software development to real-world deployment.

We provide two branches for different use cases:

- **`sim` branch**: Implements the system in a simulated environment using **Ignition Gazebo**, allowing testing and development without hardware.
- **`real` branch**: Deploys the same architecture on a physical **TurtleBot4** platform, integrating real sensors and actuators.

.. image:: /images/framework-of-rsp.jpg
   :alt: Framework of Patrolling Trash-Collecting Robot
   :align: center
   :width: 800px
   :height: 450px

.. _hardware_used:


.. _software_env:
Software Environment
---------------------

The project was developed and tested with the following software environment:

- **Ubuntu**: 22.04
- **ROS2**: Humble
- **Ignition Gazebo**: 6.16.0
- **Ignition Services**: 11.4.1
- **Python**: 3.10.12
- **C++**: 11.4.0
- **Eigen3**: 3.4.0

Hardware Used
-------------

The real-robot system is built using the following hardware:

- **Mobile Base**: TurtleBot4 (iRobot Create3 chassis)
- **Manipulator**: OpenManipulator-X (by Robotis)
- **Sensors**:
  - **Lidar (2D)**: Integrated with TurtleBot4
  - **RGBD Camera**: Intel RealSense D435 (or equivalent)
- **Onboard Computer**: Raspberry Pi 4 (provided with TurtleBot4), or external PC via Wi-Fi
- **Power Supply**: Battery integrated into TurtleBot4


.. _simulation_env:

Simulation Environment
----------------------

For development and testing, the following simulation tools and packages are used:

- **Ignition Gazebo**: Full 3D physics simulation with robot model and a custom room/lab world.
- **RViz2**: 3D visualization of the robot’s perception and planning.
- **MoveIt2**: Motion planning for the OpenManipulator-X in simulation.
- **Nav2**: Autonomous navigation stack.
- **Custom URDFs**: Combining TurtleBot4, sensors, and OpenManipulator into a single model.

.. _installation:
Installation Instructions
-------------------------

Follow the steps below to install required dependencies and set up the workspace for this project.

1. Install ROS 2 Humble on Ubuntu 22.04 following the official guide.
2. Install Gazebo Ignition:

.. code-block:: bash

   sudo apt update && sudo apt install -y \
       ros-humble-ros-ign-bridge \
       ros-humble-ros-ign-gazebo \
       ros-humble-ros-ign-image \
       ros-humble-ros-ign-gazebo-demos

3. Install MoveIt2 and related dependencies:

.. code-block:: bash

   sudo apt install ros-humble-moveit* \
       ros-humble-joint-state-publisher-gui \
       ros-humble-rqt* \
       ros-humble-xacro

4. Clone the repository and build the workspace:

.. code-block:: bash

   mkdir -p ~/rsp_ws/src
   cd ~/rsp_ws/src
   git clone https://github.com/jhu-rsp/rsp-project-team-emrs

   cd ~/rsp_ws
   source /opt/ros/humble/setup.bash
   rosdep init && rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --symlink-install
   source install/setup.bash

5. Choose a branch for your use case:

.. code-block:: bash

   cd ~/rsp_ws/src/rsp-proj
   git checkout sim   # or 'real' for hardware setup

Running Examples
________________

In the following sections, we provide examples of how to run the system in both simulation and real robot environments. The commands are categorized into different functionalities such as navigation, mapping, manipulation, and more.
A complete demo, which contains two steps,  for this project are listed in the following:

1. **Patrol**: The robot autonomously patrols a designated area, if it detects target, it will stop and wait for the next step.

.. raw:: html

    <iframe width="100%" height="450" src="https://www.youtube.com/embed/KnCYKSxr5mk?autoplay=1&mute=1" title="NAV TO BLOCK" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

2. **Pick And Place**: The robot will pick the object and place it on its body.

.. raw:: html

    <iframe width="100%" height="450" src="https://www.youtube.com/embed/eReHZW7ntQQ?autoplay=1&mute=1" title="NAV TO BLOCK" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>


.. _execution:

Running the System
-------------------

In the next section :ref:`runnable`, we list all the executive packages and commands to run the system. The commands are categorized into simulation and real robot execution.

.. list-table:: Executable Packages
   :widths: 20 50
   :header-rows: 1

   * - Executive
     - Function
   * - Common Setup
     - Clone the project and set the workspace
   * - Robot Description
     - Unified URDF model combining TurtleBot4 base, OpenManipulator-X, and camera
   * - Gazebo Simulation
     - Physics-based simulation in custom apartment and Wyman lab
   * - Hand-Eye Calibration
     - (leave empty or describe)
   * - SLAM
     - Real-time mapping and localization
   * - Patrol
     - Executes patrol routine
   * - Pick and Place
     - MoveIt2-based pick-and-place functionality for trash collection
