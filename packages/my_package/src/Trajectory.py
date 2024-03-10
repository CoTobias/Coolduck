#!/usr/bin/env python3

import os
import rospy
import math
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped
import matplotlib.pyplot as plt


class Trajectory(DTROS):
    coordinates = [(0.0, 0.0)]
    direction = ''
    moverX = []
    moverY = []
    Old_left_ticks = 0
    Old_right_ticks = 0
    slope = []
    end_of_track = False
    right_Ticks_change_Number = 0
    left_Ticks_change_Number = 0
    start_time = 0
    end_time = 0
    track_segments = []
    zähler = 0

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
        self.prev_theta = 0
        # distance per tick traveled
        self.distance_per_count = (2 * math.pi * self.wheel_radius) / 135

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
            Trajectory.right_Ticks_change_Number = Trajectory.right_Ticks_change_Number + 1
            Trajectory.left_Ticks_change_Number = Trajectory.left_Ticks_change_Number + 1
            # detect if end of track if no movement for more than 2 seconds (20HZ -> 20 messages / second)
            if Trajectory.right_Ticks_change_Number == 40 and Trajectory.left_Ticks_change_Number == 40:
                Trajectory.end_of_track = True
            return None
        else:
            # moving again so no end of track
            Trajectory.left_Ticks_change_Number = 0
            Trajectory.right_Ticks_change_Number = 0
            # every time we calculate something increase the end time
            Trajectory.end_time += 1
            # set old Trajectory to the updates value for next run
            Trajectory.Old_left_ticks = self._ticks_left
            Trajectory.Old_right_ticks = self._ticks_right
            # use another method solely for calculation
            return self._calculate_coordinates(left_ticks_change, right_ticks_change)
            # Track begins again or make sure that not end of track detected

    def _calculate_coordinates(self, left_ticks_change, right_ticks_change):
        # distance traveled with average distance per count calculated through distance_traveled/tick
        sl = self.distance_per_count * left_ticks_change
        sr = self.distance_per_count * right_ticks_change

        mean_distance = (sr + sl) / 2

        self.x += sr * math.cos(self.theta)
        self.y += sl * math.sin(self.theta)
        self.theta += (sr - sl) / self.wheel_distance

        # Ensure that theta is in the range [-pi, pi]
        if self.theta > math.pi:
            self.theta -= math.pi
        elif self.theta < -math.pi:
            self.theta += math.pi

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

    def analyze_track(self):
        if Trajectory.end_of_track:
            # set movement pattern
            pattern = self.get_movement_pattern()
            if pattern is not None:
                Trajectory.track_segments.append(pattern)
            Trajectory.start_time = Trajectory.end_time
            return Trajectory.track_segments

            """""
        i = 0
        while i < len(self.coordinates):
            current_coordinates = self.coordinates[i]
            self.set_movement_pattern(current_coordinates)
            if i % 40 == 39:
                track_segments.append(self.get_movement_pattern(i - 39, i))
            elif i + 1 == len(self.coordinates):
                x = i % 40
                track_segments.append(self.get_movement_pattern(i - x, i))
            i += 1
            """

    def get_movement_pattern(self):
        if Trajectory.end_time != 0:
            slope = self.calculate_slope()
            if slope != 0:
                Trajectory.slope.append(slope)
            """"
            if len(Trajectory.slope) > 1:
                if Trajectory.slope[len(Trajectory.slope) - 1] > Trajectory.slope[len(Trajectory.slope) - 2] + 4:
                    return "STRAIGHT"
                elif slope == 0:
                    return None
                else:
                    return "LEFT CURVE"
            else:
            """
            if abs(slope) > 0.4:
                return "LEFT CURVE"
            elif slope == 0:
                return None
            else:
                return "STRAIGHT"

    def calculate_slope(self):
        # calculate x and y resp.
        if Trajectory.end_time != 0:
            start_x, start_y = self.coordinates[Trajectory.start_time]
            end_x, end_y = self.coordinates[Trajectory.end_time]

            dx = end_x - start_x
            dy = end_y - start_y

            try:
                # with theta find out direction of movement: important for tan
                if - 0.8 < self.prev_theta < 0.8 or 1.8 <= self.prev_theta < 2.8:
                    slope = math.atan(dy / dx)
                else:
                    slope = math.atan(dx / dy)

                print(f"y: {dy} x: {dx} slope: {slope}")
                self.prev_theta = self.theta
                return slope
            except ZeroDivisionError:
                slope = dx
                return slope

        """""
        if direction == "EAST":
            # Marginal Slippage factor of 22.5 Degrees.
            if 22.5 < abs(angle) < 67:
                return "STRAIGHT , " + direction
            elif abs(angle) > 67:
                direction = "NORTH"
                return "LEFT CURVE, " + direction
        elif direction == "WEST":
            if abs(angle) > 135:
                direction = "SOUTH"
                return "LEFT CURVE, " + direction
            else:
                return "STRAIGHT" + direction
        elif direction == "NORTH":
            if abs(angle) > 67:
                direction = "WEST"
                return "LEFT CURVE" + direction
            elif abs(angle) < 67:
                return "STRAIGHT, " + direction
        elif direction == "SOUTH":
            if abs(angle) 

        return "Unknown Pattern"
        """
        """""
        # Calculate the angle in radians
        if dx != 0:
            angle = math.atan(dy / dx)
        else:
            angle = math.atan(dy / 0.1)
        # Convert the angle to degrees
        angle_degrees = math.degrees(angle)

        return angle_degrees
        """

    def run(self):
        # publish received tick messages every 0.05 second (20 Hz)
        rate = rospy.Rate(20)
        i = 0
        while not rospy.is_shutdown():
            if self._ticks_right is not None and self._ticks_left is not None:
                self.update()
                self.analyze_track()
                if Trajectory.end_of_track:
                    print(f"Track segments over the 10 seconds: {Trajectory.track_segments}")
                    print(Trajectory.coordinates)
                    # End of track finished
                    Trajectory.end_of_track = False
            rate.sleep()


if __name__ == '__main__':
    # create the node
    node = Trajectory(node_name='Trajectory', wheel_radius=3.3, wheel_distance=10, slippage_factor=0.95, speed=1)
    node.run()
    rospy.spin()
