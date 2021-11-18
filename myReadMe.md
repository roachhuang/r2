
sudo usermod -a -G dialout $USER
ros2 launch ros2_control_demo_bringup diffbot_system.launch.py
ros2 control list_hardware_interfaces
ros2 control list_controllers

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/diffbot_base_controller/cmd_vel_unstamped

ros2 topic pub --rate 30 /diffbot_base_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "linear:
 x: 0.7
 y: 0.0
 z: 0.0
angular:
 x: 0.0
 y: 0.0
 z: 1.0"

http://control.ros.org/ros2_control_demos/doc/index.html

http://control.ros.org/resources/presentations/2021-10_ROS_World_2021-ros2_control_The_future_of_ros_control.pdf
