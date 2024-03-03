#!/usr/bin/env python3

import os
import rospy
import math
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped



class WheelEncoderReaderNode(DTROS):

    def __init__(self, node_name, wheel_radius, wheel_distance, slippage_factor, speed):
        # initialize the DTROS parent class
        super(WheelEncoderReaderNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._left_encoder_topic = f"/{self._vehicle_name}/left_wheel_encoder_node/tick"
        self._right_encoder_topic = f"/{self._vehicle_name}/right_wheel_encoder_node/tick"

        # temporary data storage
        self._ticks_left = None
        self._ticks_right = None
        # construct subscriber
        self.sub_left = rospy.Subscriber(self._left_encoder_topic, WheelEncoderStamped, self.callback_left)
        self.sub_right = rospy.Subscriber(self._right_encoder_topic, WheelEncoderStamped, self.callback_right)


        # Visualisation stuff
        self.speed = speed
        self.wheel_radius = wheel_radius
        self.wheel_distance = wheel_distance
        self.slippage_factor = slippage_factor

        self.right_ticks_change = 0
        self.left_ticks_change = 0

        self.left_ticks_change_2 = 0
        self.right_ticks_change_2 = 0

        self.delta_x = 0  # Initialize delta_x
        self.delta_theta = 0  # Initialize delta_theta
        self.a = False

    def callback_left(self, data):
        # log general information once at the beginning
        rospy.loginfo_once(f"Left encoder resolution: {data.resolution}")
        rospy.loginfo_once(f"Left encoder type: {data.type}")
        # store data value
        self._ticks_left = data.data
        self.update(node)

    def callback_right(self, data):
        # log general information once at the beginning
        rospy.loginfo_once(f"Right encoder resolution: {data.resolution}")
        rospy.loginfo_once(f"Right encoder type: {data.type}")
        # store data value
        self._ticks_right = data.data
        self.update(node)

    def calculate_coordinates(self):

        self.right_ticks_change = self.right_ticks - self.right_ticks_change
        self.left_ticks_change = self.left_ticks - self.left_ticks_change

        # Check if the condition is met
        if self.left_ticks_change == 0 and self.right_ticks_change == 0:  # what is one of them changed
            self.a = True
            return None  # or any other default value
        else:
            # check if previous value was 0 if yes do something
            x = None
            y = None
            if self.a:
                # see if really something changed
                changeright_true = self.right_ticks - self.right_ticks_change_2
                changeleft_true = self.left_ticks - self.left_ticks_change_2

                if changeright_true == 0 and changeleft_true == 0:  # still zero
                    return None
                else:  # has been zero but changed
                    # Calculate traveled distances for each wheel with slippage factor
                    left_distance = 2 * self.wheel_radius * self.left_ticks * self.slippage_factor
                    right_distance = 2 * self.wheel_radius * self.right_ticks * self.slippage_factor

                    # Calculate linear and angular speed
                    linear_speed = (left_distance + right_distance) / 2.0
                    angular_speed = (right_distance - left_distance) / self.wheel_distance

                    # Calculate time elapsed (assuming 20 Hz rate)
                    time_elapsed = 1.0 / 20.0

                    # Accumulate the values over time

                    self.delta_x += linear_speed * time_elapsed * (self.speed / abs(self.speed))
                    self.delta_theta += angular_speed * time_elapsed

                    # Update the current position
                    x = self.delta_x * math.cos(self.delta_theta)  # Update x based on the heading angle
                    y = self.delta_x * math.sin(self.delta_theta)  # Update y based on the heading angle

                    # if not null do this
                    self.right_ticks_change = self.right_ticks
                    self.left_ticks_change = self.left_ticks

                    self.right_ticks_change_2 = self.right_ticks
                    self.left_ticks_change_2 = self.left_ticks
                    return x, y
            else:
                # Calculate traveled distances for each wheel with slippage factor
                left_distance = 2 * self.wheel_radius * self.left_ticks * self.slippage_factor
                right_distance = 2 * self.wheel_radius * self.right_ticks * self.slippage_factor

                # Calculate linear and angular speed
                linear_speed = (left_distance + right_distance) / 2.0
                angular_speed = (right_distance - left_distance) / self.wheel_distance

                # Calculate time elapsed (assuming 20 Hz rate)
                time_elapsed = 1.0 / 20.0

                # Accumulate the values over time

                self.delta_x += linear_speed * time_elapsed * (self.speed / abs(self.speed))
                self.delta_theta += angular_speed * time_elapsed

                # Update the current position
                x = self.delta_x * math.cos(self.delta_theta)  # Update x based on the heading angle
                y = self.delta_x * math.sin(self.delta_theta)  # Update y based on the heading angle

                # if not null do this
                self.right_ticks_change = self.right_ticks
                self.left_ticks_change = self.left_ticks

                self.right_ticks_change_2 = self.right_ticks
                self.left_ticks_change_2 = self.left_ticks
                return x, y

    def update(self, frame):
        result = self.calculate_coordinates()
        if result is not None:
            x, y = result
            coordinates.append((x, y))

    def run(self):
        # publish received tick messages every 0.05 second (20 Hz)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self._ticks_right is not None and self._ticks_left is not None:
                # start printing values when received from both encoders
                msg = f"Wheel encoder ticks [LEFT, RIGHT]: {self._ticks_left}, {self._ticks_right}"
                rospy.loginfo(msg)
                rospy.loginfo(self.calculate_coordinates(self))
                rospy.loginfo(self.update(self,))
            rate.sleep()



if __name__ == '__main__':
    # create the node
    node = WheelEncoderReaderNode(node_name='wheel_encoder_reader_node', wheel_radius=0.05, wheel_distance=0.2, slippage_factor = 0.95, speed=1)
    # if graph too small load again with bigger
    # Initialize coordinates list
    coordinates = [(0.0, 0.0)]
    #create class
    # run the timer in node
    node.run()
    print(coordinates)
    # keep spinning -> keeps ros running and processing callbacks
    rospy.spin()

