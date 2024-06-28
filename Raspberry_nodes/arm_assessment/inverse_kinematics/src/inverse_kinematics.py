#!/usr/bin/env python

import rospy
import tf2_ros
import numpy as np
from geometry_msgs.msg import TransformStamped
import tf
import math
from std_msgs.msg import Float32

def transform_to_matrix(transform):
    translation = np.array([transform.transform.translation.x,
                            transform.transform.translation.y,
                            transform.transform.translation.z])
    rotation = np.array([transform.transform.rotation.x,
                         transform.transform.rotation.y,
                         transform.transform.rotation.z,
                         transform.transform.rotation.w])
    
    matrix = np.eye(4)
    matrix[:3, :3] = tf.transformations.quaternion_matrix(rotation)[:3, :3]
    matrix[:3, 3] = translation
    return matrix

def matrix_to_transform(matrix):
    translation = matrix[:3, 3]
    rotation = tf.transformations.quaternion_from_matrix(matrix)
    transform = TransformStamped()
    transform.transform.translation.x = translation[0]
    transform.transform.translation.y = translation[1]
    transform.transform.translation.z = translation[2]
    transform.transform.rotation.x = rotation[0]
    transform.transform.rotation.y = rotation[1]
    transform.transform.rotation.z = rotation[2]
    transform.transform.rotation.w = rotation[3]
    return transform

def compute_inverse_kinematics(goal,orientation):
    xd = goal[0,3]
    yd = goal[1,3]
    zd = 0.35
    phi = round(orientation)
    
    
    L1 = 0.2
    L2 = 0.25
    L3 = 0.2
    L4 = 0.15
    
    theta1 = math.atan2(xd,yd)
    
    A = xd - L4*math.cos(theta1)*math.cos(phi)
    B = yd - L4*math.sin(theta1)*math.cos(phi)
    C = zd - L1 - L4*math.sin(phi)
    
    
    theta3 = math.acos((A**2 + B**2 + C**2 -L2**2 - L3**2)/(2*L2* L3))
    
    a = L3 * math.sin(theta3)
    b = L2 + L3 * math.cos(theta3)
    c = zd - L1 -L4 * math.sin(phi)
    r = math.sqrt(a**2 + b**2)
    
    if (c**2 > r**2):
        print(" Error NO ES POSIBLE ALCANZAR EL PUNTO") 
        return np.array([])
    
    theta2 = math.atan2(c, math.sqrt(r**2 - c**2)) - math.atan2(a,b)
    
    theta4 = phi - theta2 - theta3
    
    return np.array([theta1, -theta2, -theta3, theta4])
    
    

def compute_transform():
    rospy.init_node('compute_tf', anonymous=True)
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    theta1_pub = rospy.Publisher('/target_joint1', Float32, queue_size=10)
    theta2_pub = rospy.Publisher('/target_joint2', Float32, queue_size=10)
    theta3_pub = rospy.Publisher('/target_joint3', Float32, queue_size=10)
    theta4_pub = rospy.Publisher('/target_joint4', Float32, queue_size=10)

    rate = rospy.Rate(30.0)  # 10 Hz
    while not rospy.is_shutdown():
        try:
            # Lookup the transform from base to world
            transform_base_to_world = tf_buffer.lookup_transform('world', 'base_link', rospy.Time(0))

            # Lookup the transform from frame to base
            transform_frame_to_base = tf_buffer.lookup_transform('base_link', 'hand', rospy.Time(0))
            
            transform_marker_to_world = tf_buffer.lookup_transform('world', 'm_100', rospy.Time(0))

            # Convert transforms to matrices
            T_w_b = transform_to_matrix(transform_base_to_world)
            T_b_h = transform_to_matrix(transform_frame_to_base)
            T_w_m = transform_to_matrix(transform_marker_to_world)
            T_w_h = np.dot(T_w_b, T_b_h)
            
            T_m_w = np.linalg.inv(T_w_m)
            T_m_h = np.dot(T_m_w, T_w_h)
            euler = tf.transformations.euler_from_matrix(T_m_h)
            angles = compute_inverse_kinematics(T_w_m,euler[1])
            if (angles.size !=0):
                th1 = Float32()
                th2 = Float32()
                th3 = Float32()
                th4 = Float32()
                
                th1.data = angles[0]*180/math.pi
                th2.data = angles[1]*180/math.pi
                th3.data = angles[2]*180/math.pi
                th4.data = angles[3]*180/math.pi
                
                theta1_pub.publish(th1)
                theta2_pub.publish(th2)
                theta3_pub.publish(th3)
                theta4_pub.publish(th4)
                print("----------------------------------------- ")
                print("angles ",angles)
                print("orientation ",euler)
                print("marker position ",T_w_m[0,3]," ",T_w_m[1,3]," ",T_w_m[2,3])
                print("hand position ",T_w_h[0,3]," ",T_w_h[1,3]," ",T_w_h[2,3])
            
            # Compute the resulting transform matrix from frame to world
            
            
            



        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("TF lookup failed")

        rate.sleep()

if __name__ == '__main__':
    try:
        compute_transform()
    except rospy.ROSInterruptException:
        pass
