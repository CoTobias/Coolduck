import math
import time

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from packages.my_package.src.TrackCalculator import TrackCalculator


class TrajectoryCalculator:

    def __init__(self, wheel_radius, wheel_distance, slippage_factor, speed):
        self.speed = speed
        self.wheel_radius = wheel_radius
        self.wheel_distance = wheel_distance
        self.slippage_factor = slippage_factor
        self.left_ticks = 0
        self.right_ticks = 0

        self.right_ticks_change = 0
        self.left_ticks_change = 0

        self.left_ticks_change_2 = 0
        self.right_ticks_change_2 = 0

        self.delta_x = 0  # Initialize delta_x
        self.delta_theta = 0  # Initialize delta_theta
        self.a = False

    def update_ticks(self, left_ticks, right_ticks):
        # Update encoder ticks
        self.left_ticks = left_ticks
        self.right_ticks = right_ticks
        self.update(calculator)


    def calculate_coordinates(self):

        self.right_ticks_change = self.right_ticks - self.right_ticks_change
        self.left_ticks_change = self.left_ticks - self.left_ticks_change

        # Check if the condition is met
        if self.left_ticks_change == 0 and self.right_ticks_change == 0: # what is one of them changed
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

                if changeright_true == 0 and changeleft_true == 0:  #still zero
                    return None
                else:   #has been zero but changed
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

        return x, y

    def update(self, frame):
        result = self.calculate_coordinates()
        if result is not None:
            x , y = result
            coordinates.append((x, y))
            # Update scatter plot
            sc.set_offsets(coordinates)
            # Update lines
            if len(coordinates) > 1:
                line.set_data(zip(*coordinates))


if __name__ == "__main__":
    # Initialize the TrajectoryCalculator with wheel properties
    calculator = TrajectoryCalculator(wheel_radius=0.05, wheel_distance=0.2, slippage_factor=0.95, speed=1)

    # ** VISUALISATION START

    # Set up the initial plot
    fig, ax = plt.subplots()
    sc = ax.scatter([], [])
    line, = ax.plot([], [], color='blue')

    # Initialize coordinates list
    coordinates = [(0,0)]


    # instead of updating every sec with one coordinate i upate every second but with 20

    # Set up the animation

    # In this case, interval=50 means that each frame will have an interval of 50 milliseconds.
    # Since I want 20 frames per second, I set the interval to 50 milliseconds to achieve the desired update rate.
    animation = FuncAnimation(fig, calculator.update, interval=50, frames=20, repeat=False)


    calculator.update_ticks(300, 300)
    calculator.update_ticks(303, 301)
    calculator.update_ticks(305, 304)
    calculator.update_ticks(306, 306)



    # class that transforms my coordinates with array coordinates
    # it tests with approximation which tile is where.

    # google firebase
    # Connect online database to the online thing

    # Communication of phone and Duckiebot
    print(coordinates)
    track_calculator_instance = TrackCalculator(coordinates)
    track_segments = track_calculator_instance.analyze_track()
    print(f"Track segments over the 10 seconds: {track_segments}")
    # Show the plot
    plt.xlim(-10, 10)
    plt.ylim(-10, 10)
    plt.title('Real-time Coordinates Visualization with Lines')
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.grid(True)
    plt.show()
    # ** VISUALISATION END


    # if graph too small load again with bigger
