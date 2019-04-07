#!/usr/bin/env python
import rospy
from flask import Flask, render_template, request
from nanpy import (ArduinoApi, SerialManager)
from time import sleep
#from flask website
from nanpy import Stepper
import time
import RPi.GPIO as GPIO
from std_msgs.msg import String

app = Flask(__name__)           #placeholder for current module (app.py)
                                #instance of flask app
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

@app.route('/')                 #default route when user goes to the root url

def index():
    return render_template('home.html')            #rendering a template called home.html


@app.route('/<deviceName>/<action>')        
def do(deviceName, action):                  
    if (deviceName == 'night') and (action == 'on'):  #a
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
	
      
    elif (deviceName == 'night') and (action == 'off'):    #b  
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
          
    elif (deviceName == 'sunset') and (action == 'off'):  #2
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
    elif (deviceName == 'sunset') and (action == 'on'): #d
      a.analogWrite(pin_pwm[1], 255);
      a.digitalWrite(pin_phase[1],a.HIGH);
      a.analogWrite(pin_pwm[2], 255);
      a.digitalWrite(pin_phase[2],a.HIGH);
      a.analogWrite(pin_pwm[3], 255);
      a.digitalWrite(pin_phase[3],a.HIGH);
      a.analogWrite(pin_pwm[4], 255);
      a.digitalWrite(pin_phase[4],a.LOW); 
      sleep(0.4)
      a.analogWrite(pin_pwm[0], 255);
      a.digitalWrite(pin_phase[0],a.HIGH);
    elif (deviceName == '4pm') and (action == 'off'):     #DC motor
      filterSts = 'ON'
      #sleep(5)
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
      sleep(1)
      print ("Stopping motor")
      GPIO.output(Motor1A,GPIO.LOW)
      GPIO.output(Motor1B,GPIO.LOW)
      GPIO.output(Motor1E,GPIO.LOW)
      sleep(0.5)
      print ("Turning motor COUNTER CLOCKWISE")
      GPIO.output(Motor1A,GPIO.LOW)
      GPIO.output(Motor1B,GPIO.HIGH)
      GPIO.output(Motor1E,GPIO.HIGH)
      sleep(1)
      print ("Stopping motor")
      GPIO.output(Motor1A,GPIO.LOW)
      GPIO.output(Motor1B,GPIO.LOW)
      GPIO.output(Motor1E,GPIO.LOW)
      sleep(0.5)
      print ("Turning motor CLOCKWISE")
      GPIO.output(Motor1A,GPIO.HIGH)
      GPIO.output(Motor1B,GPIO.LOW)
      GPIO.output(Motor1E,GPIO.HIGH)
      sleep(10)
      print ("Stopping motor")
      GPIO.output(Motor1A,GPIO.LOW)
      GPIO.output(Motor1B,GPIO.LOW)
      GPIO.output(Motor1E,GPIO.LOW)
      print ("Turning motor CLOCKWISE")
      GPIO.output(Motor1A,GPIO.HIGH)
      GPIO.output(Motor1B,GPIO.LOW)
      GPIO.output(Motor1E,GPIO.HIGH)
      sleep(0.5)
      #sleep(5)               
          #GPIO.cleanup()
    elif (deviceName == '4pm') and (action == 'on'):    #f
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
    elif (deviceName == 'noon') and (action == 'off'): #3
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
    elif (deviceName == 'noon') and (action == 'on'): #  (TEMPORARY RELEASE)
      for j in MOTOR_AMOUNT:
           a.analogWrite(pin_pwm[j], 255);
           a.digitalWrite(pin_phase[j],a.LOW);
             #255, false to release
             #255, true to contract    
    elif (deviceName == '9am') and (action == 'on'): #i
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

    return render_template('home.html')

@app.route('/about')

def about():
    return render_template('about.html')


if __name__ == '__main__':
    app.run(host = '0.0.0.0',port = 80,debug = True)
