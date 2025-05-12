Deliverables and Execution Instructions
============

This section summarizes the **completed deliverables** of the Patrolling Trash-Collecting Robot project. Refer to the **Project Introduction** for full system overview, hardware details, and environment setup.

Simulation (`sim` branch)
--------------------------

.. list-table:: Simulation Deliverables
   :header-rows: 0

   * - .. image:: /images/placeholder.png
         :alt: Robot Description
         :align: center
         :scale: 50%
     - **Robot Description & Visualization**

       URDF includes TurtleBot4 base, OpenManipulator-X arm, LiDAR & RealSense.

       **Launch** RViz:
       .. code-block:: bash

          ros2 launch turtlebot4_manipulator_description \
            t4_manipulator_description.launch.py rviz:=true

       **Key Nodes**:
       - `robot_state_publisher`: broadcasts TF for all links
       - `joint_state_publisher_gui`: interactive joint control
       - `rviz2`: visualization

   * - .. image:: /images/hardware_architecture.png
         :alt: Robot Description
         :align: center
         :scale: 50%
     - **Ignition Gazebo Testbed**

       Simulation world with apartment & Wyman lab environments (SDF).

       **Launch** Ignition + Nav2:
       .. code-block:: bash

          ros2 launch turtlebot4_manipulator_ignition \
            t4_manipulator_ignition.launch.py localization:=true nav2:=true slam:=false

       **Key Nodes**:
       - `nav2_controller_server`: path following
       - `nav2_planner_server`: global planning
       - `lifecycle_manager_navigation`: manages Nav2 lifecycle
       - `ign_gazebo`: physics + sensor plugins

   * - .. image:: /images/gazebo_gif.gif
         :alt: Ignition Gazebo
         :align: center
         :scale: 50%
     - **Autonomous Patrolling**

       Waypoint patrol via Nav2 behavior tree:
       - Waypoints defined in `config/patrol_waypoints.yaml`
       - Behavior configured in `patrol.launch.py`

       **Execute** patrol:
       .. code-block:: bash

          ros2 launch turtlebot4_manipulator_navigation \
            patrol.launch.py

       **Key Nodes**:
       - `patrol_robot_node`: reads and publishes waypoints
       - `bt_navigator`: ROS2 Behavior Tree executor
       - `waypoint_follower`: sequential goal execution

   * - .. image:: /images/simulation.gif
         :alt: Autonomous Patrolling
         :align: center
         :scale: 50%
     - **Hand-Eye Calibration**

       Calibrated camera-to-end-effector transform using MoveIt Calibration:
       - Runs `moveit_calibration` pipeline
       - Uses ArUco marker board for target detection
       - Stores result as a static TF via `static_transform_publisher`

   * - .. image:: /images/eye-calibration.png
         :alt: Hand-Eye Calibration
         :align: center
         :scale: 50%
     - **Hardware Deployment**

       Real-world integration:
       - TurtleBot4 base, OpenManipulator-X arm
       - 2D LiDAR + Intel RealSense D435
       - Verified ROS2 drivers & sensor topics

   * - .. image:: /images/real_t4.gif
         :alt: Hardware Patrol
         :align: center
         :scale: 50%
     - **Navigation & Patrolling on Hardware**

       Identical launch to simulation:
       .. code-block:: bash

          ros2 launch turtlebot4_manipulator_navigation \
            patrol.launch.py

       **Key Nodes**:
       - Nav2 stack nodes (planner, controller, lifecycle)
       - `patrol_robot_node`
       - real sensor drivers

   * - .. image:: /images/real_world_a.gif
         :alt: Calibration Verification
         :align: center
         :scale: 50%
     - **Calibration & Verification**

       - Verified hand-eye transform on physical setup
       - Tested pick poses with MoveIt2 planning in real environment

Common Steps
------------

.. list-table:: Workspace & Branch Setup
   :header-rows: 0

   * - .. image:: /images/placeholder.png
         :alt: Workspace Setup
         :align: center
         :scale: 50%
     - **Workspace Initialization**

       .. code-block:: bash

          mkdir -p ~/rsp_ws/src && cd ~/rsp_ws/src
          git clone https://github.com/mrvgao/rsp-proj.git
          cd ~/rsp_ws
          rosdep install --from-paths src --ignore-src -r -y
          colcon build --symlink-install && source install/setup.bash

   * - .. image:: /images/placeholder.png
         :alt: Branch Selection
         :align: center
         :scale: 50%
     - **Branch Selection**

       .. code-block:: bash

          cd ~/rsp_ws/src/rsp-proj
          git checkout sim   # for simulation
          git checkout real  # for real hardware deployment

All source packages and launch configurations are available in the [`rsp-proj`](https://github.com/jhu-rsp/rsp-project-team-emrs) repository under the `sim` and `real` branches respectively. For detailed architecture and usage, see the **Project Introduction**, **Hardware**, **Mapping**, **Navigation**, and **Manipulation** chapters.
