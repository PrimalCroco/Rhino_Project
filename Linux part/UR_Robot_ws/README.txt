This workspace provides a ROS 2 environment for controlling and simulating Universal Robots.
The robot description is defined in the file urdf/ur_macro.xacro, which contains the kinematic and structural model of the robot.
The inverse kinematics configuration is specified in ur_moveit_config/srdf/ur_macro.srdf.xacro.
Additional configuration files include joint limits, physical parameters, and MoveIt planning settings.
This setup allows launching the robot in RViz, running controllers, using MoveIt for planning, and testing trajectories through provided scripts.



In order to launch the spiral researching mode of the UR10 (UR10 as an example but it works with the others) :

Terminal 1

  source ~/UR_Robot_ws/install/setup.bash                                                                                                    
  ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur10 \
  robot_ip:=192.168.0.1 \
  use_fake_hardware:=true \
  initial_joint_controller:=scaled_joint_trajectory_controller \
  launch_rviz:=false

Terminal 2

  source ~/UR_Robot_ws/install/setup.bash
  ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur10 \
  launch_rviz:=true

Terminal 3

  source ~/UR_Robot_ws/install/setup.bash
  python3 ~/UR_Robot_ws/src/myfiles/runme_trajectory.py ##You can put here every code in ~/UR_Robot_ws/src/myfiles$ 



