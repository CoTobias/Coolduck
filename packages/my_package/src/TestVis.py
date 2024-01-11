import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import random

# Function to generate random coordinates
def generate_coordinates():
    return random.uniform(0, 10), random.uniform(0, 10)

# Function to update the plot with new coordinates and lines
def update(frame):
    x, y = generate_coordinates()
    coordinates.append((x, y))

    # Update scatter plot
    sc.set_offsets(coordinates)

    # Update lines
    if len(coordinates) > 1:
        line.set_data(zip(*coordinates))

# Set up the initial plot
fig, ax = plt.subplots()
sc = ax.scatter([], [])
line, = ax.plot([], [], color='blue')

# Initialize coordinates list
coordinates = []

# Set up the animation
animation = FuncAnimation(fig, update, interval=1000, frames=100, repeat=False)  # Adjust frames as needed

# Show the plot
plt.xlim(0, 10)
plt.ylim(0, 10)
plt.title('Real-time Coordinates Visualization with Lines')
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.grid(True)
plt.show()
