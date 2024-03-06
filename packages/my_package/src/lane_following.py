#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped

import cv2
import numpy as np
from cv_bridge import CvBridge


class LaneFollowingNode(DTROS):

# pip3 install opencv-python numpy duckietown_msgs

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(LaneFollowingNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self._wheels_cmd_topic = f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd"
        # bridge between OpenCV and ROS
        self._bridge = CvBridge()
        # construct subscribers
        self.sub_camera = rospy.Subscriber(self._camera_topic, CompressedImage, self.camera_callback)
        # construct publisher
        self.pub_wheels_cmd = rospy.Publisher(self._wheels_cmd_topic, WheelsCmdStamped, queue_size=1)

    def camera_callback(self, msg):
        # convert JPEG bytes to CV image
        image = self._bridge.compressed_imgmsg_to_cv2(msg)

        # Apply image processing for lane detection
        processed_image = process_image(image)

        # Calculate steering command
        steering_cmd = calculate_steering(processed_image)

        # Publish control command
        self.publish_control(steering_cmd)

    def publish_control(self, steering_cmd):
        # Create a WheelsCmdStamped message
        wheels_cmd_msg = WheelsCmdStamped()
        wheels_cmd_msg.header.stamp = rospy.Time.now()
        wheels_cmd_msg.vel_left = 0.3  # adjust the velocity as needed
        wheels_cmd_msg.vel_right = 0.3  # adjust the velocity as needed
        wheels_cmd_msg.wheels_cmd.angular.z = steering_cmd
        self.pub_wheels_cmd.publish(wheels_cmd_msg)


def process_image(image):
    # Convert to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blur
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Apply Canny edge detection
    edges = cv2.Canny(blurred, 50, 150)

    # Define a region of interest (ROI)
    height, width = edges.shape
    roi_vertices = np.array([[(0, height), (width / 2, height / 2), (width, height)]], dtype=np.int32)
    roi = region_of_interest(edges, roi_vertices)

    return roi


def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    cv2.fillPoly(mask, vertices, 255)
    masked_img = cv2.bitwise_and(img, mask)
    return masked_img


def calculate_steering(image):
    # Apply Hough Transform for line detection
    lines = cv2.HoughLinesP(image, 1, np.pi / 180, threshold=20, minLineLength=20, maxLineGap=300)

    # Placeholder for steering angle
    steering_angle = 0.0

    if lines is not None:
        # Extract coordinates of line endpoints
        line_coords = np.array([[(line[0][0], line[0][1]), (line[0][2], line[0][3])] for line in lines])

        # Fit a quadratic curve to the line coordinates
        curve_fit = np.polyfit(line_coords[:, :, 0].flatten(), line_coords[:, :, 1].flatten(), 2)

        # Calculate the steering angle based on the curve fit
        steering_angle = np.arctan(2 * curve_fit[0] * line_coords[0, 0, 0] + curve_fit[1]) * 180 / np.pi

    return steering_angle


if __name__ == '__main__':
    # create the node
    node = LaneFollowingNode(node_name='lane_following_node')
    # keep spinning
    rospy.spin()
