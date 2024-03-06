#!/usr/bin/env python3

import os
import rospy
import math
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped


class Trajectory(DTROS):
    coordinates = [(0.0, 0.0)]
    track_segments = ''
    direction = ''
    moverX = []
    moverY = []
    Old_left_ticks = 0
    Old_right_ticks = 0

    def __init__(self, node_name, wheel_radius, wheel_distance, slippage_factor, speed):
        # initialize the DTROS parent class
        super(Trajectory, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
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

        self.x = 0
        self.y = 0
        self.theta = 0
        # distance per tick traveled
        self.distance_per_count = 2 * math.pi * self.wheel_radius / 135

        self.firstRight_tick = None
        self.firstLeft_tick = None

    def callback_left(self, data):
        # log general information once at the beginning
        rospy.loginfo_once(f"Left encoder resolution: {data.resolution}")
        rospy.loginfo_once(f"Left encoder type: {data.type}")
        # store data value
        if self._ticks_left is not None:
            self._ticks_left = data.data - self.firstLeft_tick
        else:
            self._ticks_left = data.data
            self.firstLeft_tick = self._ticks_left
    def callback_right(self, data):
        # log general information once at the beginning
        rospy.loginfo_once(f"Right encoder resolution: {data.resolution}")
        rospy.loginfo_once(f"Right encoder type: {data.type}")
        # store data value
        if self._ticks_right is not None:
            self._ticks_right = data.data - self.firstRight_tick
        else:
            self._ticks_right = data.data
            self.firstRight_tick = self._ticks_right

    def update(self):
        # call the update encoder and calculation method indirectly
        result = self.update_encoder_ticks()
        if result is not None:
            x, y = result
            Trajectory.coordinates.append((x, y))

    def update_encoder_ticks(self):
        left_ticks_change = self._ticks_left - Trajectory.Old_left_ticks
        right_ticks_change = self._ticks_right - Trajectory.Old_right_ticks

        if left_ticks_change == 0 and right_ticks_change == 0:
            return None
        else:
            # set old Trajectory to the updates value for next run
            Trajectory.Old_left_ticks = self._ticks_left
            Trajectory.Old_right_ticks = self._ticks_right
            # use another method solely for calculation
            return self._calculate_coordinates(left_ticks_change, right_ticks_change)

    def _calculate_coordinates(self, left_ticks_change, right_ticks_change):
        # distance traveled with average distance per count calculated through distance_traveled/tick
        sl = self.distance_per_count * left_ticks_change
        sr = self.distance_per_count * right_ticks_change

        self.x += (sr + sl) * math.cos(self.theta)
        self.y += (sr + sl) * math.sin(self.theta)
        self.theta += (sr - sl) / self.wheel_distance

        # Ensure that theta is in the range [-pi, pi]
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi
        return self.x, self.y

    """""
    def calculate_coordinates(self):

        self.right_ticks_change = self._ticks_right - self.right_ticks_change
        self.left_ticks_change = self._ticks_left - self.left_ticks_change

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
                changeright_true = self._ticks_right - self.right_ticks_change_2
                changeleft_true = self._ticks_left - self.left_ticks_change_2

                if changeright_true == 0 and changeleft_true == 0:  # still zero
                    return None
                else:  # has been zero but changed
                    # Calculate traveled distances for each wheel with slippage factor
                    left_distance = 2 * self.wheel_radius * self._ticks_left * self.slippage_factor
                    right_distance = 2 * self.wheel_radius * self._ticks_right * self.slippage_factor

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
                    self.right_ticks_change = self._ticks_right
                    self.left_ticks_change = self._ticks_left

                    self.right_ticks_change_2 = self._ticks_right
                    self.left_ticks_change_2 = self._ticks_left
                    return x, y
            else:
                # Calculate traveled distances for each wheel with slippage factor
                left_distance = 2 * self.wheel_radius * self._ticks_left * self.slippage_factor
                right_distance = 2 * self.wheel_radius * self._ticks_right * self.slippage_factor

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
                self.right_ticks_change = self._ticks_right
                self.left_ticks_change = self._ticks_left

                self.right_ticks_change_2 = self._ticks_right
                self.left_ticks_change_2 = self._ticks_left
                return x, y
    """

    def calculate_angle(self, start_time, end_time):
        dx = Trajectory.moverX[end_time] - Trajectory.moverX[start_time]
        dy = Trajectory.moverY[end_time] - Trajectory.moverY[start_time]

        # Calculate the angle in radians
        if dx != 0:
            angle = math.atan(dy / dx)
        else:
            angle = math.atan(dy / 0.1)
        # Convert the angle to degrees
        angle_degrees = math.degrees(angle)

        return angle_degrees

    def set_movement_pattern(self, current_coordinates):
        current_x, current_y = current_coordinates
        # array of movement of x and y resp.
        Trajectory.moverX.append(current_x)
        Trajectory.moverY.append(current_y)

    def get_movement_pattern(self, start_time, end_time, direction):
        angle = self.calculate_angle(start_time, end_time)
        angle_margin = 10

        if direction == "EAST":
            if abs(angle) <= angle_margin:
                return "STRAIGHT , " + direction
            elif angle > angle_margin + 90:
                return "RIGHT CURVE , " + direction
            elif angle < -angle_margin + 90:
                return "LEFT CURVE"
        elif direction == "WEST":
            if abs(angle) <= angle_margin:
                return "STRAIGHT"
            elif angle > angle_margin:
                return "LEFT CURVE"
            elif angle < -angle_margin:
                return "RIGHT CURVE"
        elif direction == "NORTH":
            if abs(angle) <= angle_margin:
                return "STRAIGHT"
            elif angle > angle_margin:
                return "RIGHT CURVE"
            elif angle < -angle_margin:
                return "LEFT CURVE"
        elif direction == "SOUTH":
            if abs(angle) <= angle_margin:
                return "STRAIGHT"
            elif angle > angle_margin:
                return "LEFT CURVE"
            elif angle < -angle_margin:
                return "RIGHT CURVE"

        return "Unknown Pattern"

    def analyze_track(self):
        track_Segments = []
        i = 0
        while i < len(self.coordinates):
            current_coordinates = self.coordinates[i]
            self.set_movement_pattern(current_coordinates)
            if i % 20 == 19:
                track_Segments.append(self.get_movement_pattern(i - 19, i, Trajectory.direction))
            elif i + 1 == len(self.coordinates):
                x = i % 20
                track_Segments.append(self.get_movement_pattern(i - x, i, Trajectory.direction))
            i += 1
        return track_Segments

    def run(self):
        # publish received tick messages every 0.05 second (20 Hz)
        rate = rospy.Rate(30)
        i = 0
        while not rospy.is_shutdown():
            if self._ticks_right is not None and self._ticks_left is not None:
                # start printing values when received from both encoders
                i = i + 1
                self.update()
                print(self._ticks_left, self._ticks_right)
                # track_segments = self.analyze_track()
                if i == 40:
                    # print(f"Track segments over the 10 seconds: {track_segments, Trajectory.direction}")
                    print(Trajectory.coordinates)
                    i = 0
            rate.sleep()


if __name__ == '__main__':
    # create the node
    node = Trajectory(node_name='Trajectory', wheel_radius=3.25, wheel_distance=6.7, slippage_factor=0.95, speed=1)
    node.run()
    rospy.spin()
