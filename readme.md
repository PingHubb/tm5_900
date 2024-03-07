# Connection:
1. ros2 run tm_driver tm_driver robot_ip:=10.0.0.53 -p port:=5890    ----   robot 1
2. ros2 run tm_driver tm_driver robot_ip:=10.0.0.43 -p port:=5890    ----   robot 2

# Command to use:
1. ros2 launch tm_moveit_cpp_demo tm5-900_run_moveit_cpp.launch.py <-- First
2. ros2 run tf2_ros tf2_echo base link_6 <--- check the Translation [Tx Ty Tz] and the Rotation: in Quaternion [x, y, z, w]
3. ros2 topic echo joint_states
4. colcon build --packages-select robot_control

