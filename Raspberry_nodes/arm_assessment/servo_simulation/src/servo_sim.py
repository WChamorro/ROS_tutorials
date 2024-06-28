#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import math

class IncrementerNode:
    def __init__(self):
        rospy.init_node('servos_node', anonymous=True)

        self.current_value_j1 = 0.0
        self.target_value_j1 = 0.0
        self.increment_j1 = 0.1  
        
        self.current_value_j2 = 0.0
        self.target_value_j2 = 0.0
        self.increment_j2 = 0.1 
        
        self.current_value_j3 = 0.0
        self.target_value_j3 = 0.0
        self.increment_j3 = 0.1 
        
        self.current_value_j4 = 0.0
        self.target_value_j4 = 0.0
        self.increment_j4 = 0.1 

        self.pubj1 = rospy.Publisher('/angle_joint1', Float32, queue_size=10)
        self.subj1 = rospy.Subscriber('/target_joint1', Float32, self.callbackj1)
        
        self.pubj2 = rospy.Publisher('/angle_joint2', Float32, queue_size=10)
        self.subj2 = rospy.Subscriber('/target_joint2', Float32, self.callbackj2)
        
        self.pubj3 = rospy.Publisher('/angle_joint3', Float32, queue_size=10)
        self.subj3 = rospy.Subscriber('/target_joint3', Float32, self.callbackj3)
        
        self.pubj4 = rospy.Publisher('/angle_joint4', Float32, queue_size=10)
        self.subj4 = rospy.Subscriber('/target_joint4', Float32, self.callbackj4)

        self.rate = rospy.Rate(10)  # 10Hz

    def callbackj1(self, msg):
        if -180.0 <= msg.data <= 180.0:
            self.target_value_j1 = math.radians(msg.data)
            if self.target_value_j1 > self.current_value_j1:
                self.increment_j1 = abs(self.increment_j1)
            elif self.target_value_j1 < self.current_value_j1:
                self.increment_j1 = -abs(self.increment_j1)
 
    def callbackj2(self, msg):
        if -180.0 <= msg.data <= 180.0:
            self.target_value_j2 = math.radians(msg.data)
        if self.target_value_j2 > self.current_value_j2:
            self.increment_j2 = abs(self.increment_j2)
        elif self.target_value_j2 < self.current_value_j2:
            self.increment_j2 = -abs(self.increment_j2)
    
    def callbackj3(self, msg):
        if -180.0 <= msg.data <= 180.0:
            self.target_value_j3 = math.radians(msg.data)
        if self.target_value_j3 > self.current_value_j3:
            self.increment_j3 = abs(self.increment_j3)
        elif self.target_value_j3 < self.current_value_j3:
            self.increment_j3 = -abs(self.increment_j3)

    def callbackj4(self, msg):
        if -180.0 <= msg.data <= 180.0:
            self.target_value_j4 = math.radians(msg.data)
        if self.target_value_j4 > self.current_value_j4:
            self.increment_j4 = abs(self.increment_j4)
        elif self.target_value_j4 < self.current_value_j4:
            self.increment_j4 = -abs(self.increment_j4)
    
                

    def run(self):
        while not rospy.is_shutdown():
            if self.current_value_j1 != self.target_value_j1:
                self.current_value_j1 += self.increment_j1
                # Ensure the current value does not overshoot the target value
                if (self.increment_j1 > 0 and self.current_value_j1 > self.target_value_j1) or (self.increment_j1 < 0 and self.current_value_j1 < self.target_value_j1):
                    self.current_value_j1 = self.target_value_j1

                # Ensure the current value stays within the range of -90 to 90 degrees (converted to radians)
                self.current_value_j1 = max(min(self.current_value_j1, math.radians(90)), math.radians(-90))
                self.pubj1.publish(self.current_value_j1)
                print("servo j1: ",self.current_value_j1)
                
            if self.current_value_j2 != self.target_value_j2:
                self.current_value_j2 += self.increment_j2
                # Ensure the current value does not overshoot the target value
                if (self.increment_j2 > 0 and self.current_value_j2 > self.target_value_j2) or (self.increment_j2 < 0 and self.current_value_j2 < self.target_value_j2):
                    self.current_value_j2 = self.target_value_j2

                # Ensure the current value stays within the range of -90 to 90 degrees (converted to radians)
                self.current_value_j2 = max(min(self.current_value_j2, math.radians(90)), math.radians(-90))
                self.pubj2.publish(self.current_value_j2)
                print("servo j2: ",self.current_value_j2)
            
            if self.current_value_j3 != self.target_value_j3:
                self.current_value_j3 += self.increment_j3
                # Ensure the current value does not overshoot the target value
                if (self.increment_j3 > 0 and self.current_value_j3 > self.target_value_j3) or (self.increment_j3 < 0 and self.current_value_j3 < self.target_value_j3):
                    self.current_value_j3 = self.target_value_j3

                # Ensure the current value stays within the range of -90 to 90 degrees (converted to radians)
                self.current_value_j3 = max(min(self.current_value_j3, math.radians(90)), math.radians(-90))
                self.pubj3.publish(self.current_value_j3)
                print("servo j3: ",self.current_value_j3)
                
            if self.current_value_j4 != self.target_value_j4:
                self.current_value_j4 += self.increment_j4
                # Ensure the current value does not overshoot the target value
                if (self.increment_j4 > 0 and self.current_value_j4 > self.target_value_j4) or (self.increment_j4 < 0 and self.current_value_j4 < self.target_value_j4):
                    self.current_value_j4 = self.target_value_j4

                # Ensure the current value stays within the range of -90 to 90 degrees (converted to radians)
                self.current_value_j4 = max(min(self.current_value_j4, math.radians(90)), math.radians(-90))
                self.pubj4.publish(self.current_value_j4)
                print("servo j4: ",self.current_value_j4)
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = IncrementerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

