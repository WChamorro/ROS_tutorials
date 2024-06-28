#!/usr/bin/env python

import rospy
import cv2
from cv2 import aruco
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
##
import tf2_ros
import geometry_msgs.msg
import tf.transformations
#

class ArucoDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_rect", Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber("/usb_cam/camera_info", CameraInfo, self.camera_info_callback)
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
        self.parameters = aruco.DetectorParameters_create()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.marker_length = 0.04  # Marker length in meters
        ##
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        ##listener of tf
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    # Camera info callback*******************************************************
    def camera_info_callback(self, camera_info):
        self.camera_matrix = np.array(camera_info.K).reshape((3, 3))
        self.dist_coeffs = np.array(camera_info.D)
        
    
    def publish_marker_tf(self, rvec, tvec,marker_id):
        marker_tf = geometry_msgs.msg.TransformStamped()
        marker_tf.header.stamp = rospy.Time.now()
        marker_tf.header.frame_id = "cam" # Parent
        marker_tf.child_frame_id = f"marker_{marker_id}"
        marker_tf.transform.translation.x = tvec[0][0]
        marker_tf.transform.translation.y = tvec[0][1]
        marker_tf.transform.translation.z = tvec[0][2]
        
        rmat, _ = cv2.Rodrigues(rvec)
        #compute rotation quaternion
        T_c_m = np.array([
            [rmat[0][0], rmat[0][1], rmat[0][2], tvec[0][0]],
            [rmat[1][0], rmat[1][1], rmat[1][2], tvec[0][1]],
            [rmat[2][0], rmat[2][1], rmat[2][2], tvec[0][2]],
            [0, 0, 0, 1]
            ])
        
        print("T_c_m",T_c_m)
        quat = tf.transformations.quaternion_from_matrix(T_c_m)
        marker_tf.transform.rotation.x = quat[0]
        marker_tf.transform.rotation.y = quat[1]
        marker_tf.transform.rotation.z = quat[2]
        marker_tf.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform(marker_tf) #Publish T_w_c
        
        try:
            # Transform from marker to world
            #Escuchar la transformada estatica de la camar con respecto al mundo T_w_c
            trans = self.tf_buffer.lookup_transform("world", "cam", rospy.Time(0))
            
            t_w_c = trans.transform.translation
            q_w_c = trans.transform.rotation
            T_w_c = tf.transformations.quaternion_matrix([q_w_c.x, q_w_c.y,  q_w_c.z,  q_w_c.w])
            T_w_c[:3, 3] = [t_w_c.x, t_w_c.y, t_w_c.z]           
            print("T_w_c",T_w_c)
           
            # Transformada del marker con respecto al mundo
            T_w_m = np.dot(T_w_c,T_c_m)
            q_w_m = tf.transformations.quaternion_from_matrix(T_w_m)
            
            print("T_w_m",T_w_m)

            # Broadcast the transform from the marker to the world frame
            t_world = geometry_msgs.msg.TransformStamped()
            t_world.header.stamp = rospy.Time.now()
            t_world.header.frame_id = "world"
            t_world.child_frame_id = f"m_{marker_id}"
            t_world.transform.translation.x = T_w_m[0,3]
            t_world.transform.translation.y = T_w_m[1,3]
            t_world.transform.translation.z = T_w_m[2,3]
            t_world.transform.rotation.x = q_w_m[0]
            t_world.transform.rotation.y = q_w_m[1]
            t_world.transform.rotation.z = q_w_m[2]
            t_world.transform.rotation.w = q_w_m[3]

            self.tf_broadcaster.sendTransform(t_world)
        except Exception as e:
            rospy.logwarn(f"Could not transform marker to world frame: {e}")
        
    # Image callback*************************************************************    
    def image_callback(self, data):
        if self.camera_matrix is None or self.dist_coeffs is None:
            rospy.logwarn("Waiting for camera_info...")
            return

        try:
            # Since the image is already in grayscale, we specify the encoding as "mono8"
            gray_image = self.bridge.imgmsg_to_cv2(data, "mono8")
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge Error: {e}")
            return
    # Marker detection<<<<<<<<
        corners, ids, rejected = aruco.detectMarkers(gray_image, self.aruco_dict, parameters=self.parameters)
        
        if ids is not None:
            aruco.drawDetectedMarkers(gray_image, corners, ids)
            
            # Estimate pose of each marker<<<< Devuelve un vector con traslacion y orientacion como angulos de euler de todos los markers
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.camera_matrix, self.dist_coeffs)
            
            # Draw axis for each marker
            for rvec, tvec, marker_id in zip(rvecs, tvecs,ids.flatten()):
                aruco.drawAxis(gray_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.marker_length * 0.5)
                self.publish_marker_tf(rvec, tvec, marker_id)

        else:
            rospy.loginfo("No markers detected")

        cv2.imshow("Aruco tag detection", gray_image)
        cv2.waitKey(3)

if __name__ == '__main__':
    rospy.init_node('aruco_detector', anonymous=True)
    aruco_detector = ArucoDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    cv2.destroyAllWindows()

