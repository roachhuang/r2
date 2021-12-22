lsusb
sudo usermod -a -G dialout $USER
ros2 launch ros2_control_demo_bringup diffbot_system.launch.py

ros2_control CLI:
    ros2 control list_hardware_interfaces
    ros2 control list_controllers
    ros2 control list_controller_types

PI:
    sudo dd if=tb3_rpi4_foxy_20210825.img of=/dev/sdb bs=4M conv=fsyncl
    gparted
    sudo -i
    nautilus (to copy 50-cloud-init.yaml to sd card's /etc/netplan)
    
    sudo apt-get ros-foxy-ros-control
    sudo apt-get ros-foxy-ros-controllers 
    rosdep install -i --from-path src --rosdistro foxy -y
    rosdep install --from-paths src --ignore-src -r -y
    colcon build --symlink-install   
    colcon build --packages-select <pkg>
    . install/setup.bash
    ros2 launch ros2_control_demo_bringup diffbot_system.launch.py

DEBUG:
    top -i (sorting keys: P, M, N, T)
    ssh <user>@<ip> sudo -c "shutdown -rf now" 
    why ros2 multicast not woring between pc & pi?
        log into adsl modem (cht/40wqe33f)
        go to advanced settings->LAN->enable IGMP LAN to LAN Multicast->Apply/Save

    ros2 run py_pubsub talker
    ros2 run py_pubsub listener

    ros2 run demo_nodes_cpp talker
    ros2 run demo_nodes_cpp listener

    ros2 run examples_rclcpp_minimal_publisher publisher_member_function

    ip link set wlan0 down

    ros2 multicast receive
    ros2 multicast send

    sudo netstat -unlp
    route add -net 224.0.0.0 netmask 240.0.0.0 dev lo

    echo $ROS_DOMAIN_ID
    echo $RMW_IMPLEMENTATION
    env | grep ROS_

On your Raspberry Pi, just install the core packages, and run only the core nodes of your applications, which are responsible for talking to the hardware. Then, on your other (remote) computer, start any simulation tool such as RViz, Gazebo. Start your heavy nodes such as motion planning, etc.

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/diffbot_base_controller/cmd_vel_unstamped

sudo apt-get install ros-<ros-distribution>-rqt-robot-steering
ros2 run rqt_robot_steering rqt_robot_steering --force-discover
ros2 topic pub --rate 30 /diffbot_base_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "linear:
 x: 0.1
 y: 0.0
 z: 0.0
angular:
 x: 0.0
 y: 0.0
 z: 0.0"

http://control.ros.org/ros2_control_demos/doc/index.html

http://control.ros.org/resources/presentations/2021-10_ROS_World_2021-ros2_control_The_future_of_ros_control.pdf

int32_t encoder[i] // in arduino
ppr=2400
r=0.0325m
2*pi*r = 0.2m
2147483648/ppr = the robot can run 894,784 revolutions (ie. ~179km) before overflow
signed number from -2,147,483,648 to 2,147,483,647

URDF:
    rosrun xacro <file>.xacro > <file>.urdf

TT motor: 
rmb:31
10000rpm at 3v; 20000rpm at 6v (+/- 10%)
1:90 (speed redution ratio)
so actual rpm = 10000/90 = 111 @ 3v 
RPM = K x PW (255 in my case)
110 = K x 255
V=2piR/60 * RPM = 2piR/60 * K * PW (R:wheel radian)


max working votage 7.4v
150ma, torque = 1.5N.m
one rotation 1170 pairs of pulse (N=1170x2=2340)
motor drive:
TB6612FNHG (input 6~12v /output to arduino 3.3 or 5v)

my chefbot:
pi camera: rmb 82.8
mpu-6050: rmb 8.79
pi 5v/3a battery(10000mAh) - rmb119
37GB520 - 4
JGB37-520 (12v,110rpm, 1:90, 11 pulses/rotation x 90 x2=1980) - rmb40
37GB-385 (1:30, 12v) -rmb59
pins (1:vin+, 2:vin-, 3:encoder gnd, 4: encoder vin(3.3~5v),5:phase A outpu, 6:phaseB)

ID549XW-5031 (12~24v, 3100rpm@12v, 6100rpm@24v, )
