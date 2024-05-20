# ROS Melodic Publisher-Subscriber Setup with Docker

This guide will walk you through setting up ROS Melodic in Docker containers on two computers, where one computer will run a publisher node and the other will run a subscriber node.

## Prerequisites

- Two computers connected to the same wireless network.
- Docker installed on both computers.

## Steps

### 1. Pull ROS Melodic Docker Image

On both computers, pull the ROS Melodic Docker image:

```bash
docker pull ros:melodic-ros-core
```

This command downloads the ROS Melodic core image from Docker Hub, which contains the essential components to run ROS.

### 2. On both computers, run the Docker container with the network set to host:

```bash 
sudo docker run -it --name ros_container --network host ros:melodic-ros-core
```

This command starts a Docker container named ros_container with network settings configured to host, allowing ROS nodes to communicate over the network.

### 3. Open another terminal and access the running container:

```
sudo docker exec -it ros_container bash
```

This command opens a bash shell inside the running Docker container, allowing you to execute commands within the container.

## Install ROS Dependencies
### 4. On both terminals, and on both computers, run the following commands:

```
sudo apt update
sudo apt install ros-melodic-catkin python-catkin-tools build-essential nano net-tools
source /opt/ros/melodic/setup.bash

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```

This updates the package lists for Ubuntu and  installs ROS Catkin build tools, Python Catkin tools, and other essential build packages along with the nano text editor. We then source the ROS setup file to set up the environment for ROS Melodic. 

## Create a New Publisher Package
### 5. On both computers and on both terminals, run:

```
cd ~/catkin_ws/src
catkin_create_pkg simple_pub_sub std_msgs rospy roscpp
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

Let's create a Catkin workspace, build it, and source the setup file to set up the workspace environment.
We then create a new ROS package named simple_pub_sub with dependencies on std_msgs, rospy, and roscpp. The workspace is then built, and the setup file is sourced.

## Computer 1 Only: Create the Python Script for the Publisher
### 6. On computer 1, run:

```
cd ~/catkin_ws/src/simple_pub_sub/src
touch simple_publisher.py
chmod +x simple_publisher.py
```

These commands navigate to the package's source directory, create a Python script file named simple_publisher.py, and make it executable.

### 7. Edit simple_publisher.py with the following content:

```
#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 1 Hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```

This script creates a ROS node named talker that publishes a "hello world" message with the current time to the chatter topic at 1 Hz.

### 8. Find the WI-FI IP address of your Docker machine. Our port will be 11311:
```
ifconfig
```

This command displays the IP address of the Docker machine.

### 9. Set the ROS_HOSTNAME, ROS_IP, ROS_MASTER_URI on both terminals:

```
export ROS_MASTER_URI=http://<IP_ADDRESS>:11311
export ROS_IP=<IP_ADDRESS>
export ROS_HOSTNAME=<IP_ADDRESS>
```

Replace <IP_ADDRESS> with your actual IP address. These commands set the ROS environment variables to configure the network settings.

### 10. Replace <IP_ADDRESS> with your actual IP address. Run ROS Master:

```
roscore
```

This command starts the ROS master, which coordinates communication between ROS nodes.

### 11. On the second terminal, run the publisher file:

```
source ~/catkin_ws/devel/setup.bash
rosrun simple_pub_sub simple_publisher.py
```

These commands source the setup file and run the publisher node.

## Computer 2 Only: Create the Python Script for the Subscriber
### 1. On computer 2, run:

```
cd ~/catkin_ws/src/simple_pub_sub/src
touch simple_subscriber.py
chmod +x simple_subscriber.py
```

These commands navigate to the package's source directory, create a Python script file named simple_subscriber.py, and make it executable.

### 2. Edit simple_subscriber.py with the following content:

```
#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.data)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('chatter', String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
```

This script creates a ROS node named listener that subscribes to the chatter topic and logs the received messages.

### 3. Find the WI-FI IP address of your Docker machine. Our port will be 11311:

```
ifconfig
```

### 4. On both terminals: set the ROS_MASTER_URI to the WI-FI IP address of your Computer 1, set the ROS_HOSTNAME and ROS_IP to the WI-FI IP address of your Computer 2.

```
export ROS_MASTER_URI=http://<IP_ADDRESS>:11311
export ROS_IP=<IP_ADDRESS>
export ROS_HOSTNAME=<IP_ADDRESS>
```

Replace <IP_ADDRESS> with your actual IP address. Make sure to replace ROS_MASTER_URI Wi-FI IP address with WI-FI IP address of Computer 1. The <IP_ADDRESS> in ROS_IP and ROS_HOSTNAME fields should be set to Computer 2's WI-FI IP address.
These commands set the ROS environment variables to configure the network settings.


### 5. Source and run the subscriber:

```
source ~/catkin_ws/devel/setup.bash
rosrun simple_pub_sub simple_subscriber.py
```

These commands source the setup file and run the subscriber node.

