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
touch simple_publisher.py && chmod +x simple_publisher.py && \
echo \"#!/usr/bin/env python\n\nimport rospy\nfrom std_msgs.msg import String\n\ndef talker():\n    pub = rospy.Publisher('chatter', String, queue_size=10)\n    rospy.init_node('talker', anonymous=True)\n    rate = rospy.Rate(1) # 1 Hz\n    while not rospy.is_shutdown():\n        hello_str = \\\"hello world %s\\\" % rospy.get_time()\n        rospy.loginfo(hello_str)\n        pub.publish(hello_str)\n        rate.sleep()\n\nif __name__ == '__main__':\n    try:\n        talker()\n    except rospy.ROSInterruptException:\n        pass\" > simple_publisher.py && \
ip_address=\$(hostname -I | awk '{print \$1}') && \
echo \"export ROS_MASTER_URI=http://\$ip_address:11311\" >> ~/.bashrc && \
echo \"export ROS_IP=\$ip_address\" >> ~/.bashrc && \
echo \"export ROS_HOSTNAME=\$ip_address\" >> ~/.bashrc && \
source ~/.bashrc && \
roscore
"

