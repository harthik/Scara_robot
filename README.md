# Scara Robot Arm - Pick and Place
In this project we designed and developed a scara robot arm capable of performing pick and place tasks. Simple inverse jacobian controller is used to control the robot arm.

1) set robot to q = [0 0 0]

2) start esp32 using docker
sudo docker run -it --rm --net=host microros/micro-ros-agent:jazzy udp4 --port 8888 -v6

3) initialize q (source ros2)
ros2 topic pub /joint_angles std_msgs/msg/Int32MultiArray "data: [0, 90, 0]" -1

4) start node minimal_publisher (source ros2 and joint_pubs)
ros2 run joint_pubs motor_pub

5) publish desired position
ros2 topic pub /desired_pos geometry_msgs/msg/Point "{x: 0.15, y: 0.05, z: 0.12}" -1

[![Watch the video]](https://raw.githubusercontent.com/harthik/Scara_robot/main/IMG_4559.MOV)
