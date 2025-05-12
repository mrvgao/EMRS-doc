Deliverables
============

This section lists the **completed deliverables** of the Patrolling Trash-Collecting Robot project, organized by functionality. For system overview and environment setup, see the **Project Introduction**.

Simulated Functionality (`sim` branch)
----------------------------------------


.. _runnable:
Common Setup
-------------

**Workspace Initialization**

After the environment installing in the :ref:`installation`, create a workspace and clone the repository:

.. code-block:: bash

   mkdir -p ~/rsp_ws/src && cd ~/rsp_ws/src
   git clone https://github.com/jhu-rsp/rsp-project-team-emrs
   cd ~/rsp_ws
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --symlink-install && source install/setup.bash

**Branch Selection**

.. code-block:: bash

   cd ~/rsp_ws/src/rsp-proj
   git checkout sim   # for simulation
   git checkout real  # for hardware deployment

Then goto the directory containing src folder and run rosdep

.. code-block:: bash

   $ rosdep install --from-paths src --ignore-src -r -y


All source packages and full launch configurations are maintained in the [`rsp-proj`](https://github.com/jhu-rsp/rsp-project-team-emrs) repository under the `sim` and `real` branches respectively. Refer to individual chapters (Mapping, Navigation, Manipulation) for deeper details on each subsystem.


Robot Description & Visualization
__________________________________

.. image:: /images/robot-description.png
   :alt: Robot Description
   :align: center
   :scale: 100%

- **Description**: Unified URDF model combining TurtleBot4 base, OpenManipulator-X arm, 2D LiDAR, and Intel RealSense camera.
- **Launch Command**::

   .. code-block:: bash

      ros2 launch turtlebot4_manipulator_description t4_manipulator_description.launch.py rviz:=true

- **Key Nodes**:
  - `robot_state_publisher`: publishes TF tree for robot links
  - `joint_state_publisher_gui`: interactive joint position control
  - `rviz2`: 3D visualization of robot and sensors

Ignition Gazebo Simulation (sim branch)
______________________________________

.. image:: /images/gazebo_gif.gif
   :alt: Gazebo Simulation
   :align: center
   :scale: 100%

- **Description**: Physics-based simulation in custom apartment and Wyman lab worlds.
- **Launch Command**::

   .. code-block:: bash

    ros2 launch turtlebot4_manipulator_ignition t4_manipulator_ignition.launch.py localization:=true nav2:=true slam:=false

- **Key Nodes**:
  - `ign_gazebo`: simulation server and physics engine
  - `ros_ign_bridge`: bridges ROS2 topics/services to Ignition
  - Nav2 nodes (`controller_server`, `planner_server`, `lifecycle_manager_navigation`)

Autonomous Patrolling In Simulation
___________________________________

.. image:: /images/simulation.gif
   :alt: Patrolling Behavior
   :align: center
   :scale: 100%

- **Description**: Waypoint-based patrol using Nav2 behavior tree.
- **Parameters**: Defined in `config/patrol_waypoints.yaml`.
- **Launch Command**::

   .. code-block:: bash

      ros2 launch turtlebot4_manipulator_navigation patrol.launch.py

- **Key Nodes**:
  - `patrol_robot_node`: reads waypoints and publishes goals
  - `bt_navigator`: executes behavior tree for navigation
  - `waypoint_follower`: follows sequential goals


Real-World Deployment (`real` branch)
-------------------------------------

**Hardware Setup & Drivers**

.. image:: /images/hardware.jpg
   :alt: Physical Robot
   :align: center
   :width: 800px
   :height: 450px

- **Description**: Integration on TurtleBot4 with OpenManipulator-X, LiDAR, RealSense.
- **Driver Nodes**:
  - `turtlebot4_node`: ROS2 driver for Create3 base
  - `open_manipulator_node`: controller for arm joints
  - Sensor drivers for LiDAR and RealSense


Hand-Eye Calibration (`real` branch)
_____________________

.. image:: /images/eye-calibration.png
   :alt: Hand-Eye Calibration
   :align: center
   :scale: 50%

- **Description**: We use the same way from ASBR to calibrate end-effector to camera transform using MoveIt calibration pipeline and ArUco board.
- **Procedure**:
  1. Launch calibration pipeline via MoveIt.
  2. Capture marker poses and compute transform.
  3. Publish static TF with `static_transform_publisher`.
- **Launch Command**::

   .. code-block:: bash

      rsp-project-team-emrs/hand-eye-calib/matlab


SLAM
____

.. raw:: html

    <iframe width="100%" height="450" src="https://www.youtube.com/embed/7yhlDjgahV4?autoplay=1&mute=1" title="SLAM" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

- **Description**: Real-time SLAM using Nav2 for mapping and localization.

- **Launch Command**::

   .. code-block:: bash

      ros2 launch turtlebot4_manipulator_navigation navigate.launch.py map_name:=r_apt.yaml

Navigation & Patrolling
________________________

.. raw:: html

    <iframe width="100%" height="450" src="https://www.youtube.com/embed/bnXM05LB094?autoplay=1&mute=1" title="Patrolling" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>


- **Description**: Executes identical patrol routine on hardware.
- **Launch Command**::

   .. code-block:: bash

      ros2 launch turtlebot4_manipulator_navigation patrol.launch.py

- **Key Nodes**:
  - Full Nav2 stack (`planner_server`, `controller_server`, `lifecycle_manager_navigation`)
  - `patrol_robot_node`
  - Hardware sensors publishing to ROS2 topics

Pick and Place
________________________
- **Description**: MoveIt2-based pick-and-place functionality for trash collection.

- **Launch Command**::

   .. code-block:: bash

      ros2 launch pick_place pick_place.launch.py

.. raw:: html

    <iframe width="100%" height="450" src="https://www.youtube.com/embed/eReHZW7ntQQ?autoplay=1&mute=1" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

