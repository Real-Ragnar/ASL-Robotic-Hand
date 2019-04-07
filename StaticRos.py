#!/usr/bin/env python
import rospy
from nanpy import (ArduinoApi, SerialManager)
from time import sleep
from nanpy import Stepper
import time
import RPi.GPIO as GPIO
from std_msgs.msg import String


MOTOR_AMOUNT = [0,1,2,3,4]
try:                                            #trying to establish connection to the arduino
     connection = SerialManager(device='/dev/ttyACM0')               #automatically finds the arduino connected
     a = ArduinoApi(connection = connection)    #instance of the arduinoapi object
except:
    print("Failed to connect to Arduino")

 #* 0 = Thumb
 #* 1 = Pinky
 #* 2 = Ring
 #* 3 = Middle
 #* 4 = Index

GPIO.setmode(GPIO.BCM)

Motor1A = 16       #DC motor pins
Motor1B = 20
Motor1E = 21
 
GPIO.setup(Motor1A,GPIO.OUT)
GPIO.setup(Motor1B,GPIO.OUT)
GPIO.setup(Motor1E,GPIO.OUT)

#Set pin numbers
pin_phase = [19, 20, 21, 22, 23];   #Direction pin servo motor
pin_pwm = [3, 4, 5, 6, 9];          #PWM pin servo motor
pin_input = [14, 15, 16, 17, 18];   #Analog input servo sensor

for i in MOTOR_AMOUNT:
    a.pinMode(pin_phase[i], a.OUTPUT);
    a.pinMode(pin_pwm[i], a.OUTPUT);
    a.pinMode(pin_input[i], a.INPUT);

 
GPIO.setup(Motor1A,GPIO.OUT)
GPIO.setup(Motor1B,GPIO.OUT)
GPIO.setup(Motor1E,GPIO.OUT)
def callback(data):
 print(data.data)
 if data.data =='a':
  print("command for a")
  a.analogWrite(pin_pwm[1], 255);
  a.digitalWrite(pin_phase[1],a.HIGH);
  a.analogWrite(pin_pwm[2], 255);
  a.digitalWrite(pin_phase[2],a.HIGH);      
  a.analogWrite(pin_pwm[3], 255);
  a.digitalWrite(pin_phase[3],a.HIGH); 
  a.analogWrite(pin_pwm[4], 255);
  a.digitalWrite(pin_phase[4],a.HIGH);  
  sleep(0.25)
  a.analogWrite(pin_pwm[0], 255);
  a.digitalWrite(pin_phase[0],a.HIGH);
 elif data.data =='b':
  print("command for b")
  a.analogWrite(pin_pwm[0], 255);
  a.digitalWrite(pin_phase[0],a.HIGH);
  a.analogWrite(pin_pwm[1], 255);
  a.digitalWrite(pin_phase[1],a.LOW);
  a.analogWrite(pin_pwm[2], 255);
  a.digitalWrite(pin_phase[2],a.LOW);      
  a.analogWrite(pin_pwm[3], 255);
  a.digitalWrite(pin_phase[3],a.LOW); 
  a.analogWrite(pin_pwm[4], 255);
  a.digitalWrite(pin_phase[4],a.LOW); 
 elif data.data =='2':
  print("command for 2")
  a.analogWrite(pin_pwm[0], 255);
  a.digitalWrite(pin_phase[0],a.HIGH);
  a.analogWrite(pin_pwm[1], 255);
  a.digitalWrite(pin_phase[1],a.HIGH);
  a.analogWrite(pin_pwm[2], 255);
  a.digitalWrite(pin_phase[2],a.HIGH);   
  a.analogWrite(pin_pwm[3], 255);
  a.digitalWrite(pin_phase[3],a.LOW); 
  a.analogWrite(pin_pwm[4], 255);
  a.digitalWrite(pin_phase[4],a.LOW);  
 elif data.data =='d':
  print("command for d")
  a.analogWrite(pin_pwm[0], 255);
  a.digitalWrite(pin_phase[0],a.HIGH);
  a.analogWrite(pin_pwm[3], 255);
  a.digitalWrite(pin_phase[3],a.HIGH); 
  a.analogWrite(pin_pwm[1], 255);
  a.digitalWrite(pin_phase[1],a.HIGH); 
  a.analogWrite(pin_pwm[2], 255);
  a.digitalWrite(pin_phase[2],a.HIGH);  
 elif data.data =='hello':
  print("command for hello")
  print ("Turning motor COUNTER CLOCKWISE")
  GPIO.output(Motor1A,GPIO.LOW)
  GPIO.output(Motor1B,GPIO.HIGH)
  GPIO.output(Motor1E,GPIO.HIGH)
  sleep(0.5)
  print ("Stopping motor")
  GPIO.output(Motor1A,GPIO.LOW)
  GPIO.output(Motor1B,GPIO.LOW)
  GPIO.output(Motor1E,GPIO.LOW)
  sleep(0.5)
  print ("Turning motor CLOCKWISE")
  GPIO.output(Motor1A,GPIO.HIGH)
  GPIO.output(Motor1B,GPIO.LOW)
  GPIO.output(Motor1E,GPIO.HIGH)
  sleep(0.5)
  print ("Stopping motor")
  GPIO.output(Motor1A,GPIO.LOW)
  GPIO.output(Motor1B,GPIO.LOW)
  GPIO.output(Motor1E,GPIO.LOW)
 elif data.data =='f':
  print("command for f")
  a.analogWrite(pin_pwm[1], 255);
  a.digitalWrite(pin_phase[1],a.LOW);
  a.analogWrite(pin_pwm[2], 255);
  a.digitalWrite(pin_phase[2],a.LOW);
  a.analogWrite(pin_pwm[3], 255);
  a.digitalWrite(pin_phase[3],a.LOW);
  a.analogWrite(pin_pwm[4], 255);
  a.digitalWrite(pin_phase[4],a.HIGH); 
  sleep(0.25)
  a.analogWrite(pin_pwm[0], 255);
  a.digitalWrite(pin_phase[0],a.HIGH);
 elif data.data =='3':
  print("command for 3")
  a.analogWrite(pin_pwm[1], 255);
  a.digitalWrite(pin_phase[1],a.HIGH);
  a.analogWrite(pin_pwm[2], 255);
  a.digitalWrite(pin_phase[2],a.HIGH);
  a.analogWrite(pin_pwm[0], 255);
  a.digitalWrite(pin_phase[0],a.LOW);
  a.analogWrite(pin_pwm[3], 255);
  a.digitalWrite(pin_phase[3],a.LOW);
  a.analogWrite(pin_pwm[4], 255);
  a.digitalWrite(pin_phase[4],a.LOW);  
 elif data.data =='release':
  print("command for release")
  for j in MOTOR_AMOUNT:
       a.analogWrite(pin_pwm[j], 255);
       a.digitalWrite(pin_phase[j],a.LOW);
         #255, false to release
         #255, true to contract   
 elif data.data =='i':
  print("command for i")
  a.analogWrite(pin_pwm[2], 255);
  a.digitalWrite(pin_phase[2],a.HIGH);
  a.analogWrite(pin_pwm[3], 255);
  a.digitalWrite(pin_phase[3],a.HIGH);
  a.analogWrite(pin_pwm[1], 255);
  a.digitalWrite(pin_phase[1],a.LOW);
  a.analogWrite(pin_pwm[4], 255);
  a.digitalWrite(pin_phase[4],a.HIGH); 
  sleep(0.25)
  a.analogWrite(pin_pwm[0], 255);
  a.digitalWrite(pin_phase[0],a.HIGH);


rospy.init_node('StaticRos', anonymous=True)
sub=rospy.Subscriber('static_gests', String, callback)
rospy.spin()


