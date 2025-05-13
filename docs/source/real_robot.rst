Real-World Deployment (`real` branch)
=======================================

This section lists the **Real-World Deployment** of the Patrolling Trash-Collecting Robot project, organized by functionality. For system overview and environment setup, see the **Project Introduction**.


.. code:: bash

    cd ~/rsp_ws/src/rsp-project-team-emrs
    git checkout real   # for real robot

**Hardware and Drivers**

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

- **Setting of USB3.0**: We used USB 3.0 for the OAK-D and RealSense cameras to handle the high bandwidth needed for streaming RGB and depth data at full resolution and frame rate—something USB 2.0 couldn’t support without frame drops. Since the Raspberry Pi couldn’t supply enough power to run both cameras and the robot arm, we added a separate external power supply for the arm to ensure stable performance without overloading the Pi.

TurtleBot4 Setup
_________________

.. warning::

    Make sure the Turtlebot4 is in Access Point (AP) mode with namespace "emrs", following the instructions in https://turtlebot.github.io/turtlebot4-user-manual/setup/basic.html.
Connect to the Turtlebot4 network and access the Create3 webserver through the Raspberry Pi's ip address and port 8080.

example: 192.168.28.24:8080

 Follow the instructions here (https://turtlebot.github.io/turtlebot4-user-manual/setup/discovery_server.html) to set up Discovery Server on the Turtlebot4 and get its topics on your own PC.

SLAM - Generating a map
_______________________

Check that you can see the Turtlebot4 topics on your PC by running

.. code:: bash

    ros2 topic list

You should see topics multiple topics with prefix "emrs". If you do not, check that you are connected to the AP network, run "source /etc/turtlebot4_discovery/setup.bash", "ros2 daemon stop", and "ros2 daemon start". Check the topic list and ensure that they are being published to by echo-ing or checking the hz. Finally, you can run the following command:

.. code:: bash

    ros2 launch turtlebot4_navigation slam.launch.py namespace:=emrs
    ros2 launch turtlebot4_viz view_robot.launch.py namespace:=emrs

Connect the teleop controller and drive the robot around the room, making sure the map on Rviz is continuously growing. Once satisfied with the map, save it by calling

.. code:: bash

    ros2 run nav2_map_server map_saver_cli -f "map_name" --ros-args -p map_subscribe_transient_local:=true -r __ns:=/emrs

And move the pgm and yaml file to the maps folder in the turtlebot4_manipulator_navigation package.

.. raw:: html

    <iframe width="100%" height="450" src="https://www.youtube.com/embed/7yhlDjgahV4?autoplay=1&mute=1" title="SLAM" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>


Localization, Navigation, and Patrolling
________________________________________

Note: the localization command may not properly run if the robot has been running for a while. In order to guarantee success of the command, restart the Turtlebot4 and make sure you are receiving its topics.
Now you can run localization:

.. code:: bash

    ros2 launch turtlebot4_manipulator_navigation navigate.py map_name:=wyman_157_hd.yaml

changing the map name to whatever you saved your SLAM map as. Once Rviz opens and the map appears, use Rviz to give an initial 2D pose. The robot model, laser scan, costmap/voxels should appear. Once you see the voxels, you can input Nav2 goals, taking note of the position and orientation to save later as a patrol route.
Once you are satisfied, edit the "patrol_waypoints.yaml" file with your waypoints, where patrol_route should be a flattened array with every 6 elements corresponding to, x, y, z, qx, qy, qz, qw of each waypoint.
Once a patrol route is saved, restart the Turtlebot4 and you can now run the patrol.
In order to run the patrol, run the navigate command above, wait for the voxels to appear, then run:


.. code:: bash

    ros2 launch turtlebot4_manipulator_navigation patrol.py

You should see the robot start to move in the patrol route you saved earlier.


.. raw:: html

    <iframe width="100%" height="450" src="https://www.youtube.com/embed/bnXM05LB094?autoplay=1&mute=1" title="Patrolling" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>



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

    cd ~/rsp_ws/src/rsp-project-team-emrs
    git checkout real
    matlab -nodisplay -nodesktop -r "run('~/rsp_ws/src/rsp-project-team-emrs/hand_eye_calib/matlab/hand_eye_calib.mlx')"


Pick and Place
________________________
- **Description**: MoveIt2-based pick-and-place functionality for trash collection.

- **Launch Command**:: To run the pick and place action, ssh into the Turtlebot4 by running:

.. code-block:: bash

    ssh -X ubuntu@10.42.0.1

replacing "10.42.0.1" with the ip address shown on the Turtlebot4 screen. Repeat this for a total of three terminals each ssh'ed into the robot.
Check which USB port the arm is plugged into by running:

.. code-block:: bash

    sudo dmesg | grep tty

and then add permissions,

.. code-block:: bash

    chmod 666 /dev/ttyUSB0

replacing "/dev/ttyUSB0" with whatever port name you have. Also run

.. code-block:: bash

    groups

and see if the user is in the "dialout" group. If not, run:

.. code-block:: bash

    sudo usermod -aG dialout $USER

Then start the open-manipulator-x arm by running the following command, making sure to physically hold the robot arm near its home configuration before doing so:

.. code-block:: bash

    ros2 launch open_manipulator_x_bringup hardware.launch.py port_name:=/dev/ttyUSB0

.. note::

    The motors should engage and you may then let go of the arm. If there are errors, it is most likely because of the port name being wrong or permissions not being applied (may need a reboot). Then, in another terminal, run:

ros2 launch open_manipulator_x_moveit_config move_group.launch.py

Finally, run:

.. code-block:: bash

    ros2 launch pick_place pick_place.launch.py

to see a pick and place action.


.. raw:: html

    <iframe width="100%" height="450" src="https://www.youtube.com/embed/eReHZW7ntQQ?autoplay=1&mute=1" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>


Trash Actions - Approach, Pick, and Place
________________________________________

**Approach**

This action will search for an ArUco tag in the view of the OAK-D camera on the Turtlebot4 and travel in front of it to put the Open Manipulator-X arm in range of the trash object. It uses the Approach.action interface:

.. code-block:: srv

    string marker_frame        # TF frame of the detected ArUco marker
    bool success
    string message
    ---
    float64 distance_to_goal

with the goal "marker_frame" as the name of the frame outputted by the ArUco detection node. This is usually "marker".

**Pick**

This action will search for the ArUco tag of the trash object in the view of the RealSense D435 camera mounted on the Open Manipulator-X arm and will move the arm to pick up the trash object. It uses the Pick.action interface:

.. code-block:: srv

    string marker_frame        # TF frame of the detected ArUco marker
    ---
    bool success
    string message
    ---
    float64 distance_to_goal

with the goal "marker_frame" as the name of the frame outputted by the ArUco detection node. This is usually "marker_arm".

**Place**

This action will move the arm with the trash object to the trash can, opening the gripper and depositing the trash inside. It uses the Place.action interface:

.. code-block:: srv

    geometry_msgs/PoseStamped trash_pose
    ---
    bool success
    string message
    ---
    float64 distance_to_goal

with the goal "trash_pose" as the pose of the arm above the trash can.

