#!/usr/bin/env python3

import os
import rospy
import matplotlib.pyplot as plt
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped


class WheelEncoderReaderNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(WheelEncoderReaderNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._left_encoder_topic = f"/{self._vehicle_name}/left_wheel_encoder_node/tick"
        self._right_encoder_topic = f"/{self._vehicle_name}/right_wheel_encoder_node/tick"


        # construct subscriber
        self.sub_left = rospy.Subscriber(self._left_encoder_topic, WheelEncoderStamped, self.callback_left)
        self.sub_right = rospy.Subscriber(self._right_encoder_topic, WheelEncoderStamped, self.callback_right)

    def callback_left(self, data):
        # log general information once at the beginning
        rospy.loginfo_once(f"Left encoder resolution: {data.resolution}")
        rospy.loginfo_once(f"Left encoder type: {data.type}")
        # store data value
        self._ticks_left = data.data

    def callback_right(self, data):
        # log general information once at the beginning
        rospy.loginfo_once(f"Right encoder resolution: {data.resolution}")
        rospy.loginfo_once(f"Right encoder type: {data.type}")
        # store data value
        self._ticks_right = data.data

    def visualize_line(coordinates):
        # the * in front of coordinates unpacks the coordinates
        x, y = zip(*coordinates)
        plt.plot(x, y, marker='o')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Line Visualization')
        plt.grid(True)
        plt.show()

    def calculate_coordinates(self):
        #how to approximately calculate coordinates through wheel encoders
        wheel_radius = 0.1





    def run(self):
        # publish received tick messages every 0.05 second (20 Hz)
        rate = rospy.Rate(20)
        # continue as long as it is not shutdown
        while not rospy.is_shutdown():
            if self._ticks_right is not None and self._ticks_left is not None:
                # start printing values when received from both encoders
                msg = f"Wheel encoder ticks [LEFT, RIGHT]: {self._ticks_left}, {self._ticks_right}"
                rospy.loginfo(msg)




                #loop sleeps for the appropriate duration to archive the rate
                rate.sleep()

if __name__ == '__main__':
    # create the node
    node = WheelEncoderReaderNode(node_name='wheel_encoder_reader_node')
    # run the timer in node
    node.run()
    # keep spinning -> keeps ros running and processing callbacks
    rospy.spin()

