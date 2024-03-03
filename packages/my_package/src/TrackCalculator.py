import math
class TrackCalculator:
    moverX = []
    moverY = []
    lolli = [(0, 0),(0,1),(0,2),(0,3),(0,4),(0,5),(0,6),(0,7),(0,8),(0, 18), (0, 19), (0, 20), (0, 21), (0, 22), (0, 23), (0, 24), (0, 25), (0, 26), (0, 27),
    (0, 28), (0, 29), (0, 30), (0, 31), (0, 32), (0, 33), (0, 34), (0, 35), (0, 36), (0, 37),
    (0, 38), (0, 39), (0, 40), (0, 41), (0, 42), (0, 43), (0, 44), (0, 45), (0, 46), (0, 47),
    (0, 48), (0, 49), (0, 50), (0, 51), (0, 52), (0, 53), (0, 54), (0, 55), (0, 56), (0, 57),
    (0, 58), (0, 59), (0, 60), (0, 61), (0, 62), (0, 63), (0, 64), (0, 65), (0, 66), (0, 67)]

    straight_line = [(x, y) for x, y in zip(range(50), [0] * 50)]
    left_curve = [(x, y) for x, y in zip(range(50), range(50))]
    gibberish = [(x * 2, y * 3) for x, y in zip(range(50), range(50))]
    slightly_curved_straight_line = [(x, y + x // 5) for x, y in zip(range(50), [0] * 50)]
    slightly_curved_right_curve = [(x, y - x // 5) for x, y in zip(range(50), range(50))]
    slightly_curved_left_curve = [(x, y + x // 5) for x, y in zip(range(50), range(50))]

    coordinates = slightly_curved_right_curve = [(x, y - x // 5) for x, y in zip(range(50), range(50))]

    tuple_coordinates = ()
    direction = ""

    def __init__(self):
        # Initialize instance variables here if needed
        pass

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
                return "STRAIGHT"
            elif angle > angle_margin + 90:
                return "RIGHT CURVE"
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

        # direction calculation
        '''
        if TrackCalculator.moverX[start_time] < TrackCalculator.moverX[end_time] and TrackCalculator.moverY[start_time] < TrackCalculator.moverY[end_time]:
            if direction == "EAST" and angle < -angle_margin:
                # LEFT UP
                direction == "NORTH"
                return "Left Curve"
            elif direction == "WEST" and angle > angle_margin:
                direction = "NORTH"
                return "Right Curve"
            elif abs(angle) <= angle_margin:
                return "Straight"
        elif TrackCalculator.moverX[start_time] < TrackCalculator.moverX[end_time] and TrackCalculator.moverY[start_time] > TrackCalculator.moverY[end_time]:
            if direction == "SOUTH" and angle < -angle_margin:

            elif direction == "WEST" and angle > angle_margin:

            elif abs(angle) <= angle_margin:
                return "Straight"
        elif TrackCalculator.moverX[start_time] > TrackCalculator.moverX[end_time] and TrackCalculator.moverY[start_time] > TrackCalculator.moverY[end_time]:
            if direction == "EAST" and angle < -angle_margin:

            elif direction == "WEST" and angle > angle_margin:

            elif abs(angle) <= angle_margin:
                return "Straight"
        elif TrackCalculator.moverX[start_time] > TrackCalculator.moverX[end_time] and TrackCalculator.moverY[start_time] < TrackCalculator.moverY[end_time]:
            if direction == "EAST" and angle < -angle_margin:

            elif direction == "WEST" and angle > angle_margin:

            elif abs(angle) <= angle_margin:
                return "Straight"
        else:
            if abs(angle) <= angle_margin:
                return "Straight"
            elif angle > angle_margin:
                return "Right Curve"
            elif angle < -angle_margin:
                return "Left Curve"
            else:
                return "Unknown Pattern"
        '''


    def analyze_track(self):
        track_Segments = []
        direction = "EAST"
        i = 0
        while i < len(TrackCalculator.coordinates):
            current_coordinates = TrackCalculator.coordinates[i]
            print(current_coordinates)
            self.set_movement_pattern(current_coordinates)
            if i % 50 == 49:  # Changed from i == 20 to analyze 20 elements
                track_Segments.append(self.get_movement_pattern(i - 19, i, direction))
            i += 1


        return track_Segments


if __name__ == "__main__":
    track_calculator_instance = TrackCalculator()

    track_segments = track_calculator_instance.analyze_track()
    print(f"Track segments over the 10 seconds: {track_segments}")
