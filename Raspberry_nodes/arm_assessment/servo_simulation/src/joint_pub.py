#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState

class JointStatePublisher:
    def __init__(self):
        rospy.init_node('joint_state_publisher', anonymous=True)

        self.joint_state = JointState()
        self.joint_state.name = ['j1', 'j2', 'j3', 'j4']
        self.joint_state.position = [0.0, 0.0, 0.0, 0.0]

        self.pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

        self.sub1 = rospy.Subscriber('/angle_joint1', Float32, self.joint1_callback)
        self.sub2 = rospy.Subscriber('/angle_joint2', Float32, self.joint2_callback)
        self.sub3 = rospy.Subscriber('/angle_joint3', Float32, self.joint3_callback)
        self.sub4 = rospy.Subscriber('/angle_joint4', Float32, self.joint4_callback)

        self.rate = rospy.Rate(60)  # 10Hz

    def joint1_callback(self, msg):
        self.joint_state.position[0] = msg.data

    def joint2_callback(self, msg):
        self.joint_state.position[1] = msg.data

    def joint3_callback(self, msg):
        self.joint_state.position[2] = msg.data

    def joint4_callback(self, msg):
        self.joint_state.position[3] = msg.data

    def run(self):
        while not rospy.is_shutdown():
            self.joint_state.header.stamp = rospy.Time.now()
            self.pub.publish(self.joint_state)
            print("joint states: ",self.joint_state.position)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = JointStatePublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass
