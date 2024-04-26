#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import time

#Setup servoPin as PWM output of frequancy 100Hz
servoPin=12
GPIO.setmode(GPIO.BOARD)
GPIO.setup(servoPin,GPIO.OUT)
pwm=GPIO.PWM(servoPin,100)

def AngleToDuty(ang):
  return float(ang)/18.+10.

def angle_callback(msg):
    angle = msg.data
    duty = AngleToDuty(angle)
    pwm.ChangeDutyCycle(duty)
    print("angle ",angle, "duty ",duty)
    
def listener():


    rospy.init_node('servotest', anonymous=True)
    rospy.Subscriber("angle", Float32, angle_callback)
    pwm.start(5)
    rospy.spin()

if __name__ == '__main__':
    listener()
    pwm.stop() #stop sending value to output
    GPIO.cleanup() #release channel
    
