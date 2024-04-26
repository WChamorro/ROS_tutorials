
import rospy
from std_msgs.msg import Float32
import time

import time
import Adafruit_PCA9685

#Setup servoPin as PWM output of frequancy 100Hz
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(100)

def AngleToDuty(ang):
  return float(ang)/18.+10.

def angle_callback(msg):
    angle = msg.data
    duty = AngleToDuty(angle)
    on = int(duty*4096/100)
    off = int(4096-on)
    pwm.set_pwm(15, on, off)
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
    
