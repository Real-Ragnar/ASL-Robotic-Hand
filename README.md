# ASL-Robotic-Hand
Robotic Hand controlled by 6 motors connected to a raspberry pi and laptop. Laptop uses neural networks to recognize hand gestures. RPI runs a webserver.
Note: 
*Ubuntu 16.04 installed on the PC using dual boot to maximize space 
*Ubuntu 16.04 also installed on the RPI 
*ROS-Kinetic Desktop Full Install was installed on both the PC and the RPI:
http://wiki.ros.org/kinetic/Installation/Ubuntu
*after creating the nodes for static robot commander and dynamic robot commander make the scripts executable by running: 
	cd catkin_ws/ 
	catkin_make 
*for static classifier dependancies installation:
-install pip:
sudo apt-get install python-pip
-install numpy 1.16.2:
pip install numpy
-install pyQT4
https://www.saltycrane.com/blog/2008/01/how-to-install-pyqt4-on-ubuntu-linux/
-install h5py
sudo apt install python-h5py

------
On DESKTOP:
1. In terminal run sudo gedit .bashrc and add the following lines at the bottom and save:

source /opt/ros/kinetic/setup.bash
export PYTHONPATH=~/pybrain:$PYTHONPATH
source ~/catkin_ws/devel/setup.bash
export PYTHONPATH=$PYTHONPATH:$HOME/LeapSDK/lib:$HOME/LeapSDK/lib/x64
export ROS_MASTER_URI=http://localhost:11311/
export ROS_HOSTNAME=<INSERT  THE COMPUTER IP>
export ROS_IP=<INSERT THE COMPUTER IP>

2. Now run sudo gedit /etc/hosts/  to add the IP address of the RPI with its hostname 

-----
On RPI:
1. In terminal run sudo nano .bashrc and add the following lines at the bottom:
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://<IP OF COMPUTER>:11311/
export ROS_HOSTNAME=<INSERT IP of RPI>
export ROS_IP=<INSERT IP of RPI>

2. Run sudo nano /etc/hosts/ and add the IP address of the Computer with its hostname 

----
ON COMPUTER:

1.  Install leap motion package from: https://github.com/ros-drivers/leap_motion

2. Install dynamic classifier package from:http://wiki.ros.org/TanvirParhar/neuro_gesture_leap and create a subdirectory in the package where you will build your dataset. 

3.Create a new package for static classifier by following ROS tutorial section 3 : 
http://wiki.ros.org/ROS/Tutorials/CreatingPackage
*note we will use the same dependancies std_msgs rospy roscpp

4. Now download the static classifier zip file from: https://nsmith5.itch.io/leap-commander
-Unzip the file on your desktop, go to unzip folder and copy everything from the folder to the src subdirectory in your new package. Now move the files from the ui subdirectory to the src subdirectory in the new package to have all files together. Now make every .py file executable by using chmod +x file.py 
To train the classifier follow the instructions from the link above. 


5.configure your ROS nodes:
-Leap motion publisher node is already configured, dynamic classifier node is already set as a subscriber node to receive the leap motion data to predict the performed gesture.
-Set the dynamic classifier node as a publisher to send the predicted gesture to a topic to allow the robot node to capture the predicted gestures. To do so, edit the gesture_rec.py file located in the neuro_gesture_leap package in the scripts subdirectory:
Line 37: pub = rospy.Publisher('/neuro_gest/gesture',String, queue_size=10)
Line 80: pub.publish(gest_name[index])
-Set the static classifier as a node and a publisher: edit the main.py file in the static classifier package to set it as a node and also be a publisher to send the predicted gestures to the topic to allow the robot node to receive the predicted gesture:
-at the top add: 
#!/usr/bin/env python
import rospy 
from std_msgs.msg import String
-under import numpy as np add: 
pub = rospy.Publisher('static_gests', String, queue_size=10)
rospy.init_node('main', anonymous=True)
-line 151 add:
 pub.publish(command)


6. On RPI configuring the robot commander nodes:
1. In the catkin workspace on the raspberry Pi create a package where you will create the two subscriber nodes: one for executing commands from the static classifier, and the other to execute commands from the dynamic classifier. 

