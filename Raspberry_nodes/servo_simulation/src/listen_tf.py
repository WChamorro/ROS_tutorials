#!/usr/bin/env python

import rospy
import tf2_ros
import tf.transformations
from geometry_msgs.msg import TransformStamped

import tf2_geometry_msgs.tf2_geometry_msgs

def publish_transform():
    rospy.init_node('tf_listener_republisher')
    
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    rate = rospy.Rate(10.0)  # 10 Hz

    while not rospy.is_shutdown():
        try:
            # Listen for the transform from 'base_link' to 'target_frame'
            T_w_b = tf_buffer.lookup_transform('world', 'base_link', rospy.Time(0))
            T_b_tip = tf_buffer.lookup_transform("base_link", "end", rospy.Time(0))
            
            #T_w_tip = tf2_geometry_msgs.tf2_geometry_msgs.do_tr
            
            
            #tf_broadcaster.sendTransform(T_w_tip)
            
            #T = tf_buffer.lookup_transform('base_link', 'tip', rospy.Time(0))
            #t_b_tip = T.transform.translation
            #q_b_tip = T.transform.rotation
            #T_b_tip = tf.transformations.quaternion_matrix(q_b_tip.x, q_b_tip.y, q_b_tip.z, q_b_tip.w)
            #T_b_tip[:3, 3] = [t_b_tip.x, t_b_tip.y, t_b_tip.z]
        
        except Exception as e:
            rospy.logwarn("LookupException: %s", e)

            
        


        rate.sleep()

if __name__ == '__main__':
    try:
        publish_transform()
    except rospy.ROSInterruptException:
        pass
