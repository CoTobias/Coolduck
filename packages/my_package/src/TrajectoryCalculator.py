import math
import time

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import random
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

        self.delta_x = 0  # Initialize delta_x
        self.delta_theta = 0  # Initialize delta_theta

    def update_ticks(self, left_ticks, right_ticks):
        # Update encoder ticks
        self.left_ticks = left_ticks
        self.right_ticks = right_ticks

    def calculate_coordinates(self):
        self.right_ticks_change = self.right_ticks - self.right_ticks_change
        self.left_ticks_change = self.left_ticks - self.left_ticks_change

        print(self.left_ticks_change, self.right_ticks_change)

        # Calculate traveled distances for each wheel with slippage factor
        left_distance = 2 * self.wheel_radius * self.left_ticks * self.slippage_factor
        right_distance = 2 * self.wheel_radius * self.right_ticks * self.slippage_factor

        # Calculate linear and angular speed
        linear_speed = (left_distance + right_distance) / 2.0
        angular_speed = (right_distance - left_distance) / self.wheel_distance

        # Calculate time elapsed (assuming 20 Hz rate)
        time_elapsed = 1.0 / 20.0

        # Accumulate the values over time
        if self.left_ticks_change != 0 or self.left_ticks_change != 0:
            self.delta_x += linear_speed * time_elapsed * (self.speed / abs(self.speed))
            self.delta_theta += angular_speed * time_elapsed

        # Update the current position
        x = self.delta_x * math.cos(self.delta_theta)  # Update x based on the heading angle
        y = self.delta_x * math.sin(self.delta_theta)  # Update y based on the heading angle

        self.right_ticks_change = self.right_ticks
        self.left_ticks_change = self.left_ticks

        return x, y
    def update(self, frame):
        x, y = self.calculate_coordinates()
        coordinates.append((x, y))

        # Update scatter plot
        sc.set_offsets(coordinates)

        # Update lines
        if len(coordinates) > 1:
            line.set_data(zip(*coordinates))



if __name__ == "__main__":
    # Initialize the TrajectoryCalculator with wheel properties
    calculator = TrajectoryCalculator(wheel_radius=0.05, wheel_distance=0.2, slippage_factor =0.95, speed= 1)

    # ** VISUALISATION START

    # Set up the initial plot
    fig, ax = plt.subplots()
    sc = ax.scatter([], [])
    line, = ax.plot([], [], color='blue')

    # Initialize coordinates list
    coordinates = []

    # Update ticks everytime with self_left ticks etc
    calculator.update_ticks(left_ticks=100, right_ticks=102)

    calculator.update_ticks(left_ticks=110, right_ticks=120)

    calculator.update_ticks(left_ticks=115, right_ticks=130)

    calculator.update_ticks(left_ticks=150, right_ticks=165)
    time.sleep(2)
    calculator.update_ticks(left_ticks=160, right_ticks=175)

    # Set up the animation
    animation = FuncAnimation(fig, calculator.update, interval=1000, frames=100, repeat=False)  # Adjust frames as needed


    # Show the plot
    plt.xlim(-10, 10)
    plt.ylim( -10 , 10)
    plt.title('Real-time Coordinates Visualization with Lines')
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.grid(True)
    plt.show()

    # ** VISUALISATION END


