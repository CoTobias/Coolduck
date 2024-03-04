import math
class TrackCalculator:
    moverX = []
    moverY = []

    straight_line = [(x, y) for x, y in zip(range(50), [0] * 50)]
    left_curve = [(x, y) for x, y in zip(range(50), range(50))]
    gibberish = [(x * 2, y * 3) for x, y in zip(range(50), range(50))]
    slightly_curved_straight_line = [(x, y + x // 5) for x, y in zip(range(50), [0] * 50)]
    slightly_curved_right_curve = [(x, y - x // 5) for x, y in zip(range(50), range(50))]
    slightly_curved_left_curve = [(x, y + x // 5) for x, y in zip(range(50), range(50))]

    tuple_coordinates = ()
    direction = "EAST"

    def __init__(self, coordinates):
        # Initialize instance variables here if needed
        self.coordinates = coordinates;
    def calculate_angle(self, start_time, end_time):
        dx = TrackCalculator.moverX[end_time] - TrackCalculator.moverX[start_time]
        dy = TrackCalculator.moverY[end_time] - TrackCalculator.moverY[start_time]

        # Calculate the angle in radians
        angle = math.atan(dy/dx)

        # Convert the angle to degrees
        angle_degrees = math.degrees(angle)

        return angle_degrees

    def set_movement_pattern(self, current_coordinates):
        current_x, current_y = current_coordinates
        # array of movement of x and y resp.
        TrackCalculator.moverX.append(current_x)
        TrackCalculator.moverY.append(current_y)



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
                track_Segments.append(self.get_movement_pattern(i - 19, i, TrackCalculator.direction))
            elif i + 1 == len(self.coordinates):
                x = i % 20
                track_Segments.append(self.get_movement_pattern(i - x, i, TrackCalculator.direction))
            i += 1
        return track_Segments


if __name__ == "__main__":
    track_calculator_instance = TrackCalculator()
    track_segments = track_calculator_instance.analyze_track()
    print(f"Track segments over the 10 seconds: {track_segments, TrackCalculator.direction}")
