#!/usr/bin/env python

import rospy
import cv2
from cv2 import aruco
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class ArucoDetector:
    def __init__(self):
        self.bridge = CvBridge() #<<<<<<< puente para manejar la imagen enter ROS y opencv
        self.image_sub = rospy.Subscriber("/usb_cam/image_rect", Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber("/usb_cam/camera_info", CameraInfo, self.camera_info_callback)
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
        self.parameters = aruco.DetectorParameters_create()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.marker_length = 0.05  # Marker length in meters

    # Camera info callback*******************************************************
    def camera_info_callback(self, camera_info):
        self.camera_matrix = np.array(camera_info.K).reshape((3, 3))
        self.dist_coeffs = np.array(camera_info.D)
        print("Got camera info")
    
    # Image callback*************************************************************    
    def image_callback(self, data):
        if self.camera_matrix is None or self.dist_coeffs is None:
            rospy.logwarn("Waiting for camera_info...")
            return

        try:
            # Convertir la imagen de ROS en un ohjeto de open cv
            gray_image = self.bridge.imgmsg_to_cv2(data, "mono8")
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge Error: {e}")
            return
    
    # Marker detection<<<<<<<<
        corners, ids, rejected = aruco.detectMarkers(gray_image, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            aruco.drawDetectedMarkers(gray_image, corners, ids)

            # Estimacion del pose del marker con respecto a la camara<<<< Devuelve un vector con traslacion y orientacion como angulos de           euler de todos los markers
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.camera_matrix, self.dist_coeffs)

            # Draw axis for each marker
            for rvec, tvec in zip(rvecs, tvecs):
                aruco.drawAxis(gray_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.marker_length * 0.5)

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

