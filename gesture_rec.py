#!/usr/bin/env python
import rospy
import os
import pickle
import numpy as np
import csv
from leap_motion.msg import leap
from leap_motion.msg import leapros
from std_msgs.msg import String
from geometry_msgs.msg import Point
from pybrain.tools.shortcuts import buildNetwork
from pybrain.supervised.trainers import BackpropTrainer
from pybrain.datasets import SequentialDataSet, SupervisedDataSet,ClassificationDataSet
from pybrain.structure import SigmoidLayer, LinearLayer


data1=Point()
x=[]
y=[]
z=[]

mat=[]

rospy.init_node('gesture_rec', anonymous=True)

def callback_ros(data):
 #rospy.loginfo(rospy.get_name() + ": Leap ROS Data %s" % data)
 global data1
 data1=data.palmpos

def subscriber():
  
  rospy.Subscriber("leapmotion/data", leapros, callback_ros)
  
subscriber()

pub = rospy.Publisher('/neuro_gest/gesture',String, queue_size=10)

f1=open('pic1.pickle','rb')
trainer=pickle.load(f1)
n=pickle.load(f1)
gest_name=pickle.load(f1)

print "\n"
print "Welcome to the Leap to Pi Mode" 
print " Please read the instructions below before starting the game:"
print " 1.The user must perform a gesture with his right-hand."
print " 2.The matching gesture will be displayed by the robot."
print " 3.Please refer to the RPI GUI for the allowed gestures. \n" 

while True:
 x[:]=[]
 y[:]=[]
 z[:]=[]
 mat[:]=[]
 index=0

 char=raw_input("To start tracing a letter please enter 'm', to exit press anything else:  ")

 if char=="m" or char=="M":
   time=rospy.get_time()
   while rospy.get_time()<time+3.5:
     print data1
     x.append(data1.x)
     y.append(data1.y)
     z.append(data1.z)
     rospy.sleep(0.1)
   all_points=[x,y,z]
   for k in range(0,len(x)-1,1):
      mat.append(x[k+1]-x[k])
      mat.append(y[k+1]-y[k])
      mat.append(z[k+1]-z[k])
   out = n.activate(mat)
   index=np.argmax(out)
   
   
   print index
   print out
   print "You traced letter '"+gest_name[index]+"'"
   pub.publish(gest_name[index])
   
	
     
 else:
  print "Goodbye :)"
  break

