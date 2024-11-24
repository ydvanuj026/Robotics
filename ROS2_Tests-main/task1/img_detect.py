#!/usr/bin/python3
# -*- coding: utf-8 -*-

'''
*****************************************************************************************
*
*        		===============================================
*           		    Logistic coBot (LB) Theme (eYRC 2024-25)
*        		===============================================
*
*  This script should be used to implement Task 1B of Logistic coBot (LB) Theme (eYRC 2024-25).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:          [ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:		    task1b_boiler_plate.py
# Functions:
#			        [ Comma separated list of functions in this file ]
# Nodes:		    Add your publishing and subscribing node
#			        Publishing Topics  - [ /tf ]
#                   Subscribing Topics - [ /camera/aligned_depth_to_color/image_raw, /etc... ]


################### IMPORT MODULES #######################

import rclpy
import sys
import cv2
import math
import tf2_ros
import numpy as np
import cv2.aruco as aruco
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CompressedImage, Image
import tf_transformations as tf
import tf2_ros


##################### FUNCTION DEFINITIONS #######################

def calculate_rectangle_area(coordinates):
    '''
    Description:    Function to calculate area of detected ArUco marker

    Args:
        coordinates (list):     coordinates of detected ArUco marker (4 sets of (x, y) coordinates)

    Returns:
        area        (float):    area of detected ArUco marker
        width       (float):    width of detected ArUco marker
    '''
    
    ############ ADD YOUR CODE HERE ############

    # Extract each corner point
    (x1, y1) = coordinates[0]
    (x2, y2) = coordinates[1]
    (x3, y3) = coordinates[2]
    (x4, y4) = coordinates[3]

    # Calculate width (distance between points 1 and 2)
    width = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    
    # Calculate height (distance between points 1 and 4)
    height = math.sqrt((x4 - x1) ** 2 + (y4 - y1) ** 2)

    # Calculate area
    area = width * height

    ############################################

    return area, width



def calculate_rectangle_area(coordinates):
    (x1, y1) = coordinates[0]
    (x2, y2) = coordinates[1]
    (x3, y3) = coordinates[2]
    (x4, y4) = coordinates[3]
    width = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    height = math.sqrt((x4 - x1) ** 2 + (y4 - y1) ** 2)
    area = width * height
    return area, width

def detect_aruco(image):
    '''
    Function to perform ArUco detection and return each detail of ArUco detected.

    Args:
        image (Image): Input image frame received from respective camera topic.

    Returns:
        center_aruco_list (list): Center points of all ArUco markers detected.
        distance_from_rgb_list (list): Distance value of each ArUco marker detected from RGB camera.
        angle_aruco_list (list): Angle of all pose estimated for ArUco markers.
        width_aruco_list (list): Width of all detected ArUco markers.
        ids (list): List of all ArUco marker IDs detected in a single frame.
    '''
    
    # Constants and variables
    aruco_area_threshold = 1500
    cam_mat = np.array([[931.1829833984375, 0.0, 640.0], [0.0, 931.1829833984375, 360.0], [0.0, 0.0, 1.0]])
    dist_mat = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    size_of_aruco_m = 0.15
    
    center_aruco_list = []
    distance_from_rgb_list = []
    angle_aruco_list = []
    width_aruco_list = []
    ids = []

    # Convert image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Define ArUco dictionary and parameters
    aruco_dict = aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_params = aruco.DetectorParameters()

    # Detect ArUco markers
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
    
    if ids is None:
        # Return empty lists if no markers are detected
        return center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, []

    # Draw detected markers
    aruco.drawDetectedMarkers(image, corners, ids)

    for i, corner in enumerate(corners):
        marker_id = ids[i][0]
        coordinates = corner[0]

        # Calculate area and width of the detected marker
        area, width = calculate_rectangle_area(coordinates)

        # Filter markers based on area threshold
        if area < aruco_area_threshold:
            continue
        
        # Calculate center of the marker
        center_x = int(np.mean(coordinates[:, 0]))
        center_y = int(np.mean(coordinates[:, 1]))
        center_aruco_list.append((center_x, center_y))
        width_aruco_list.append(width)
        
        # Pose estimation
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers([corner], size_of_aruco_m, cam_mat, dist_mat)
        
        # Calculate distance from RGB camera
        distance = np.linalg.norm(tvec[0][0])
        distance_from_rgb_list.append(distance)
        
        # Calculate rotation angle (for simplicity, consider the rotation around z-axis)
        angle = math.degrees(math.atan2(rvec[0][0][1], rvec[0][0][0]))
        angle_aruco_list.append(angle)
        
        # Draw the pose axis on the marker
        cv2.drawFrameAxes(image, cam_mat, dist_mat, rvec, tvec, 0.1)
    
    return center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids.flatten().tolist()



##################### CLASS DEFINITION #######################

class aruco_tf(Node):
    '''
    ___CLASS___

    Description:    Class which servers purpose to define process for detecting aruco marker and publishing tf on pose estimated.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class aruco_tf
                        All classes have a function called __init__(), which is always executed when the class is being initiated.
                        The __init__() function is called automatically every time the class is being used to create a new object.
                        You can find more on this topic here -> https://www.w3schools.com/python/python_classes.asp
        '''

        super().__init__('aruco_tf_publisher')                                          # registering node

        ############ Topic SUBSCRIPTIONS ############

        self.color_cam_sub = self.create_subscription(Image, '/camera/color/image_raw', self.colorimagecb, 10)
        self.depth_cam_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depthimagecb, 10)

        ############ Constructor VARIABLES/OBJECTS ############

        image_processing_rate = 0.2                                                     # rate of time to process image (seconds)
        self.bridge = CvBridge()                                                        # initialise CvBridge object for image conversion
        self.tf_buffer = tf2_ros.buffer.Buffer()                                        # buffer time used for listening transforms
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)                                    # object as transform broadcaster to send transform wrt some frame_id
        self.timer = self.create_timer(image_processing_rate, self.process_image)       # creating a timer based function which gets called on every 0.2 seconds (as defined by 'image_processing_rate' variable)
        
        self.cv_image = None                                                            # colour raw image variable (from colorimagecb())
        self.depth_image = None                                                         # depth image variable (from depthimagecb())


    def depthimagecb(self, data):
        '''
        Description:    Callback function for aligned depth camera topic. 
                        Use this function to receive image depth data and convert to CV2 image

        Args:
            data (Image):    Input depth image frame received from aligned depth camera topic

        Returns:
        '''
        try:
            # Convert ROS 2 Depth Image message to OpenCV image
            # Adjust encoding based on your sensor's depth image encoding
            self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

            # Normalize the depth image for display purposes (convert to 8-bit image)
            depth_display = cv2.normalize(self.depth_image, None, 0, 255, cv2.NORM_MINMAX)
            depth_display = np.uint8(depth_display)

            # Display the depth image
            # cv2.imshow("Depth Image Window", depth_display)
            # cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Could not convert depth image: {e}")

        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP : 

        #	->  Use data variable to convert ROS Image message to CV2 Image type

        #   ->  HINT: You may use CvBridge to do the same

        ############################################


    def colorimagecb(self, data):
        '''
        Description:    Callback function for colour camera raw topic.
                        Use this function to receive raw image data and convert to CV2 image

        Args:
            data (Image):    Input coloured raw image frame received from image_raw camera topic

        Returns:
        '''
        try:
            # Convert ROS 2 Image message to OpenCV image
            self.cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
            # 
            # Display the OpenCV image
            # cv2.imshow("Image Window", self.cv_image)
            # cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Could not convert image: {e}")

        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP : 

        #	->  Use data variable to convert ROS Image message to CV2 Image type

        #   ->  HINT:   You may use CvBridge to do the same
        #               Check if you need any rotation or flipping image as input data maybe different than what you expect to be.
        #               You may use cv2 functions such as 'flip' and 'rotate' to do the same

        ############################################


    def process_image(self):
        '''
        Function used to detect ArUco markers and publish transforms (TF) for estimated poses.
        '''

        # Camera info parameters
        sizeCamX = 1280
        sizeCamY = 720
        centerCamX = 640 
        centerCamY = 360
        focalX = 931.1829833984375
        focalY = 931.1829833984375
        
        # Detect ArUco markers and get relevant data
        center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids = detect_aruco(self.depth_image)
        
        if not ids:
            return  # No markers detected

        # Initialize tf broadcaster
        tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        for i, marker_id in enumerate(ids):
            cX, cY = center_aruco_list[i]
            distance_from_rgb = distance_from_rgb_list[i]
            angle_aruco = angle_aruco_list[i]

            # Correct the ArUco angle using the given formula
            angle_corrected = (0.788 * angle_aruco) - ((angle_aruco ** 2) / 3160)

            # Calculate quaternion from yaw angle (roll and pitch are 0)
            quaternion = tf.quaternion_from_euler(0, 0, math.radians(angle_corrected))

            # Calculate the real-world position of the marker
            x = distance_from_rgb * (sizeCamX - cX - centerCamX) / focalX
            y = distance_from_rgb * (sizeCamY - cY - centerCamY) / focalY
            z = distance_from_rgb/1000  # Convert mm to meters

            # Draw the center point on the image
            cv2.circle(self.cv_image, (cX, cY), 5, (0, 255, 0), -1)

            # Publish transform from camera_link to marker
            transform_camera = TransformStamped()
            transform_camera.header.stamp = self.get_clock().now().to_msg()
            transform_camera.header.frame_id = 'camera_link'
            transform_camera.child_frame_id = f'cam_{marker_id}'
            transform_camera.transform.translation.x = x
            transform_camera.transform.translation.y = y
            transform_camera.transform.translation.z = z
            transform_camera.transform.rotation.x = quaternion[0]
            transform_camera.transform.rotation.y = quaternion[1]
            transform_camera.transform.rotation.z = quaternion[2]
            transform_camera.transform.rotation.w = quaternion[3]
            tf_broadcaster.sendTransform(transform_camera)

            # Lookup transform between base_link and camera_link
            try:
                trans = self.tf_buffer.lookup_transform('base_link', transform_camera.child_frame_id, rclpy.time.Time())
                transform_base = TransformStamped()
                transform_base.header.stamp = self.get_clock().now().to_msg()
                transform_base.header.frame_id = 'base_link'
                transform_base.child_frame_id = f'obj_{marker_id}'
                transform_base.transform = trans.transform  # Copy over the translation and rotation
                tf_broadcaster.sendTransform(transform_base)
            except Exception as e:
                self.get_logger().warn(f"Failed to lookup transform: {e}")

        # Display the result
        cv2.imshow("Detected ArUco Markers", self.cv_image)
        cv2.waitKey(1)



##################### FUNCTION DEFINITION #######################

def main():
    '''
    Description:    Main function which creates a ROS node and spin around for the aruco_tf class to perform it's task
    '''

    rclpy.init(args=sys.argv)                                       # initialisation

    node = rclpy.create_node('aruco_tf_process')                    # creating ROS node

    node.get_logger().info('Node created: Aruco tf process')        # logging information

    aruco_tf_class = aruco_tf()                                     # creating a new object for class 'aruco_tf'

    rclpy.spin(aruco_tf_class)                                      # spining on the object to make it alive in ROS 2 DDS

    aruco_tf_class.destroy_node()                                   # destroy node after spin ends

    rclpy.shutdown()                                                # shutdown process


if __name__ == '__main__':
    '''
    Description:    If the python interpreter is running that module (the source file) as the main program, 
                    it sets the special __name__ variable to have a value “__main__”. 
                    If this file is being imported from another module, __name__ will be set to the module’s name.
                    You can find more on this here -> https://www.geeksforgeeks.org/what-does-the-if-__name__-__main__-do/
    '''

    main()
