#!/bin/bash

# Pull the ROS image and run the container
docker pull ros:melodic-ros-core
docker run -d --name ros_container --network host ros:melodic-ros-core

# Install necessary packages inside the container
docker exec -it ros_container bash -c "
apt update && apt install -y ros-melodic-catkin python-catkin-tools build-essential nano && \
source /opt/ros/melodic/setup.bash && \
mkdir -p ~/catkin_ws/src && \
cd ~/catkin_ws/ && \
catkin_make && \
source devel/setup.bash && \
cd ~/catkin_ws/src && \
catkin_create_pkg simple_pub_sub std_msgs rospy roscpp && \
cd ~/catkin_ws && \
catkin_make && \
source devel/setup.bash && \
cd ~/catkin_ws/src/simple_pub_sub/src && \
touch simple_subscriber.py && chmod +x simple_subscriber.py && \
echo \"#!/usr/bin/env python\n\nimport rospy\nfrom std_msgs.msg import String\n\ndef callback(data):\n    rospy.loginfo(rospy.get_caller_id() + \\\" I heard %s\\\" % data.data)\n\ndef listener():\n    rospy.init_node('listener', anonymous=True)\n    rospy.Subscriber('chatter', String, callback)\n    rospy.spin()\n\nif __name__ == '__main__':\n    listener()\" > simple_subscriber.py && \
ip_address=\$(hostname -I | awk '{print \$1}') && \
echo \"export ROS_MASTER_URI=http://<Computer_1_IP>:11311\" >> ~/.bashrc && \
echo \"export ROS_IP=\$ip_address\" >> ~/.bashrc && \
echo \"export ROS_HOSTNAME=\$ip_address\" >> ~/.bashrc && \
source ~/.bashrc && \
source ~/catkin_ws/devel/setup.bash && \
rosrun simple_pub_sub simple_subscriber.py
"

