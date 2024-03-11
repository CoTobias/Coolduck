#!/usr/bin/env python3

import os
import rospy
import math
from std_msgs.msg import String
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped
import matplotlib.pyplot as plt


class Trajectory(DTROS):
    # Duckiebot starts at Origin with direction "East"
    coordinates = [(0.0, 0.0)]
    # Graph that will look like the track the bot drives on -> Also starts at Origin with same direction
    track_coordinates = [(0.0, 0.0)]
    # Sequence of tracks traversed
    track_segments = []
    # used for Change caluclation
    prev_left_ticks = 0
    prev_right_ticks = 0
    # Indicates if we have finished a track: (2 seconds passed without tick change in any wheel)
    end_of_track = False
    # used for end_of_track calculation
    right_Ticks_change_Number = 0
    left_Ticks_change_Number = 0
    # gets incremented for track detection
    start_time = 0
    end_time = 0
    # Important for calculation as slope calc. differs from east-west movement to North-South movement
    east_west = True

    def __init__(self, node_name, wheel_radius, wheel_distance, slippage_factor, speed):
        # Initialize the DTROS parent class
        super(Trajectory, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        # Vehicle name
        self._vehicle_name = os.environ['VEHICLE_NAME']
        # Left and Right Encoder respectively
        self._left_encoder_topic = f"/{self._vehicle_name}/left_wheel_encoder_node/tick"
        self._right_encoder_topic = f"/{self._vehicle_name}/right_wheel_encoder_node/tick"
        # Subscriber for the Encoders
        self.sub_left = rospy.Subscriber(self._left_encoder_topic, WheelEncoderStamped, self.callback_left)
        self.sub_right = rospy.Subscriber(self._right_encoder_topic, WheelEncoderStamped, self.callback_right)
        # Publisher used for publishing the Information for Mobile Application
        self._publisher = rospy.Publisher('chatter', String, queue_size=10)
        # Value of Encoders at current point in time
        self._ticks_left = None
        self._ticks_right = None
        # Wheel Radius of the Duckiebot: Changes will impact the whole graph
        self.wheel_radius = wheel_radius
        # Distance between the wheels, not initialized here but in the constructor of our node
        self.wheel_distance = wheel_distance
        # Coordinate x and y at current moment in time
        self.x = 0
        self.y = 0
        # Previous x and y coordinates
        self.prev_x = 0
        self.prev_y = 0
        # Theta is used for orientation (direction) of the bot
        self.theta = 0
        self.prev_theta = 0
        # Distance per tick traveled -> 135 ticks per Rotation
        self.distance_per_count = (2 * math.pi * self.wheel_radius) / 135
        # The Duckiebot does not return to zero with the value of each encoder ocne it has started. To avoid having
        # to start with a coordinate at 1000... or having to calculate extensivly I just subtract the first value
        # from the encoder value and get 0 as starting position then
        self.firstRight_tick = None
        self.firstLeft_tick = None

    # partly taken from official Duckietown Documentation:
    # https://docs.duckietown.com/daffy/devmanual-software/beginner/ros/wheel-encoder-reader.html
    def callback_left(self, data):
        # Log general information once at the beginning to make sure everything works properly
        rospy.loginfo_once(f"Left encoder resolution: {data.resolution}")
        rospy.loginfo_once(f"Left encoder type: {data.type}")
        # General Case -> used to get Encoder Start value to 0
        if self._ticks_left is not None:
            self._ticks_left = data.data - self.firstLeft_tick
        # First time calculation of Encoder
        else:
            self._ticks_left = data.data
            self.firstLeft_tick = self._ticks_left

    # partly taken from official Duckietown Documentation:
    # https://docs.duckietown.com/daffy/devmanual-software/beginner/ros/wheel-encoder-reader.html
    def callback_right(self, data):
        # Log general information once at the beginning to make sure everything works properly
        rospy.loginfo_once(f"Right encoder resolution: {data.resolution}")
        rospy.loginfo_once(f"Right encoder type: {data.type}")
        # General Case -> used to get Encoder Start value to 0
        if self._ticks_right is not None:
            self._ticks_right = data.data - self.firstRight_tick
        # First time calculation of Encoder
        else:
            self._ticks_right = data.data
            self.firstRight_tick = self._ticks_right

    # Method that called in the run method -> Calls different methods to find out coordinates at current pos.
    def update(self):
        # Call the update encoder and calculation method indirectly
        result = self.update_encoder_ticks()
        # Just when change happens coordinates get appended, or else we would have 1000 points at the same value
        if result is not None:
            x, y = result
            Trajectory.coordinates.append((x, y))

    # Method that occupies with change of encoder value
    def update_encoder_ticks(self):
        left_ticks_change = self._ticks_left - Trajectory.prev_left_ticks
        right_ticks_change = self._ticks_right - Trajectory.prev_right_ticks
        # Logic: When nothing changes for > 2 seconds for either encoder -> Track is finished
        if left_ticks_change == 0 and right_ticks_change == 0:
            Trajectory.right_Ticks_change_Number = Trajectory.right_Ticks_change_Number + 1
            Trajectory.left_Ticks_change_Number = Trajectory.left_Ticks_change_Number + 1
            # Detect if end of track if no movement for more than 2 seconds (20HZ -> 20 messages / second)
            if Trajectory.right_Ticks_change_Number == 40 and Trajectory.left_Ticks_change_Number == 40:
                Trajectory.end_of_track = True
            return None
        # Logic: If stuff changes it is on the track and moves
        else:
            # Moving again so no end of track
            Trajectory.left_Ticks_change_Number = 0
            Trajectory.right_Ticks_change_Number = 0
            # Every time we calculate something increase the end time
            Trajectory.end_time += 1
            # Set old Trajectory to the updates value for next run
            Trajectory.prev_left_ticks = self._ticks_left
            Trajectory.prev_right_ticks = self._ticks_right
            # Use another method solely for calculation
            return self._calculate_coordinates(left_ticks_change, right_ticks_change)
            # Track begins again or make sure that not end of track detected

    # Actual Calculation of coordinates of robot with differential steering:
    # Elementary Trajectory Model for the Differential Steering System of Robot Wheel Actuators
    # Source: https://rossum.sourceforge.net/papers/DiffSteer/
    def _calculate_coordinates(self, left_ticks_change, right_ticks_change):
        # distance traveled with average distance per count calculated through distance_traveled/tick
        sl = self.distance_per_count * left_ticks_change
        sr = self.distance_per_count * right_ticks_change * 1.03

        mean_distance = (sr + sl) / 2

        self.x += mean_distance * math.cos(self.theta)
        self.y += mean_distance * math.sin(self.theta)
        self.theta += (sr - sl) / self.wheel_distance

        # Ensure that theta is in the range [-pi, pi]
        # Ensure that theta is in the range [-pi, pi]
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

        return self.x, self.y

    def analyze_track(self):
        if Trajectory.end_of_track:
            # set movement pattern
            pattern = self.get_movement_pattern()
            if pattern is not None:
                Trajectory.track_segments.append(pattern)
                self.make_track(pattern)
            Trajectory.start_time = Trajectory.end_time
            return Trajectory.track_segments

    def make_track(self, pattern):
        start_x, start_y = self.coordinates[Trajectory.start_time]
        end_x, end_y = self.coordinates[Trajectory.end_time]

        if Trajectory.end_of_track:
            # set movement pattern
            if pattern is not None:
                if pattern == "STRAIGHT":
                    if Trajectory.east_west:
                        # only x coordinate changes
                        line = self.generate_straight_line(start_x, start_y, end_x, start_y, 50)
                        Trajectory.track_coordinates.extend(line)
                    else:
                        line = self.generate_straight_line(start_x, start_y, start_x, end_y, 50)
                        Trajectory.track_coordinates.extend(line)
                    # assign values
                    self.prev_y = start_y
                    self.prev_x = end_x
                elif pattern == "LEFT CURVE":
                    # adjust distance to adjust curvature
                    distance = 25
                    # Calculate midpoint of the line segment
                    mid_x = (self.prev_x + end_x) / 2
                    mid_y = (self.prev_y + end_y) / 2

                    # Calculate the angle of the line segment
                    angle = math.atan2(end_y - self.prev_y, end_x - self.prev_x)

                    # Calculate control point
                    control_x = mid_x + distance * math.cos(angle - math.pi / 2)
                    control_y = mid_y + distance * math.sin(angle - math.pi / 2)

                    curve_points = self.generate_bezier_curve(self.prev_x, self.prev_y, end_x, end_y, control_x,
                                                              control_y, 80)
                    Trajectory.track_coordinates.extend(curve_points)

                    # assign last value tp prev value
                    self.prev_x = end_x
                    self.prev_y = end_y
        print(Trajectory.track_coordinates)
        print("----------------------------------------------------------------------------------")

    def generate_bezier_curve(self, start_x, start_y, end_x, end_y, control_x, control_y, num_points):
        # Generate points on the Bézier curve
        curve_points = []
        for t in [i / (num_points - 1) for i in range(num_points)]:
            x = (1 - t) ** 2 * start_x + 2 * (1 - t) * t * control_x + t ** 2 * end_x
            y = (1 - t) ** 2 * start_y + 2 * (1 - t) * t * control_y + t ** 2 * end_y
            curve_points.append((x, y))

        return curve_points

    def generate_straight_line(self, start_x, start_y, end_x, end_y, num_points):
        # Calculate the slope
        slope = (end_y - start_y) / (end_x - start_x) if (end_x - start_x) != 0 else float('inf')

        # Calculate the y-intercept
        y_intercept = start_y - slope * start_x

        # Generate points along the line
        line_points = []
        for i in range(num_points):
            x = start_x + i * (end_x - start_x) / (num_points - 1)
            y = slope * x + y_intercept
            line_points.append((x, y))

        return line_points


    def get_movement_pattern(self):
        if Trajectory.end_time != 0:
            slope = self.calculate_slope()
            if abs(slope) > 0.3:
                return "LEFT CURVE"
            # right curve just without abs. and with east west check
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
                if -0.5 < self.prev_theta < 0.3 or 1.8 <= self.prev_theta < 2.3 or -math.pi < self.prev_theta < -2.5 or 2.9 < self.prev_theta < math.pi:
                    Trajectory.east_west = True
                    slope = math.atan(dy / dx)
                else:
                    Trajectory.east_west = False
                    slope = math.atan(dx / dy)
                # print(f"y: {dy} x: {dx} slope: {slope}")
                print(f"slope {slope}" f" y {dy}" f" x {dx}" f" theta {self.prev_theta}")
                self.prev_theta = self.theta
                return slope
            except ZeroDivisionError:
                slope = dx
                return slope

    def run(self):
        # publish received tick messages every 0.05 second (20 Hz)
        rate = rospy.Rate(20)
        i = 0
        while not rospy.is_shutdown():
            if self._ticks_right is not None and self._ticks_left is not None:
                self.update()
                self.analyze_track()
                message = f"{Trajectory.coordinates}!"
                self._publisher.publish(message)
                if Trajectory.end_of_track:
                    print(f"Track segments traversed: {Trajectory.track_segments}")
                    print(Trajectory.coordinates)
                    print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
                    # End of track finished
                    Trajectory.end_of_track = False
            rate.sleep()


if __name__ == '__main__':
    # create the node
    node = Trajectory(node_name='Trajectory', wheel_radius=3.3, wheel_distance=10, slippage_factor=0.95, speed=1)
    node.run()
    rospy.spin()
    #Trajectory.coordinates.append((0,0))
    #print(Trajectory.coordinates)
