import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import math

# Define car parameters
turning_radius = 5.0  # Adjust as needed
wheelbase = 1.5  # Adjust as needed
max_steering_angle = np.deg2rad(30)  # Maximum steering angle in radians

# Define waypoints
waypoints = np.array([[0, 0], [5, 5], [10, 0]])

# Create Dubins Path
def dubins_path(start, end, turning_radius):
    # Dubins path generation logic
    # Implement your Dubins path generation here
    # ...
    # Calculate the difference in orientation between the start and end points
    delta_theta = np.arctan2(end[1] - start[1], end[0] - start[0])

    # Calculate the distance between the start and end points
    delta_x = end[0] - start[0]
    delta_y = end[1] - start[1]
    delta = np.sqrt(delta_x**2 + delta_y**2)
    print("delta: ", delta)

    # Check if the start and end points are too close
    if delta < 2 * turning_radius:
        raise ValueError("Start and end points are too close for the given turning radius")

    # Check if the turning radius is smaller than half the distance
    if turning_radius < delta / 2:
        raise ValueError("Turning radius is too small for the given start and end points")

    # Calculate the minimum turning radius for a clothoid segment
    min_clothoid_radius = delta**2 / (8 * turning_radius) + turning_radius / 2

    # Check if the turning radius is smaller than the minimum clothoid radius
    if turning_radius < min_clothoid_radius:
        raise ValueError("Turning radius is too small for the given start and end points")

    # Calculate the length of the clothoid segment
    clothoid_length = min_clothoid_radius * delta_theta

    # Calculate the length of the circle arc segment
    circle_arc_length = delta - 2 * clothoid_length

    # Check if the Dubins path is feasible
    if circle_arc_length < 0:
        raise ValueError("Dubins path is not feasible with the given turning radius")

    # Calculate the Dubins path as a sequence of segments (CSC, CCC, C, etc.)
    dubins_segments = []

    # Add the first clothoid segment (CSC or CCSC)
    if clothoid_length > 0:
        if turning_radius < min_clothoid_radius:
            dubins_segments.append(("C", clothoid_length))
        else:
            dubins_segments.append(("CC", clothoid_length))
            dubins_segments.append(("C", clothoid_length))

    # Add the circle arc segment (L or R depending on delta_theta)
    if delta_theta > 0:
        dubins_segments.append(("L", circle_arc_length))
    else:
        dubins_segments.append(("R", circle_arc_length))

    # Add the second clothoid segment (CSC or CCSC)
    if clothoid_length > 0:
        dubins_segments.append(("C", clothoid_length))
        if turning_radius < min_clothoid_radius:
            dubins_segments.append(("CC", clothoid_length))

    return dubins_segments

# Check if path is traversable
def is_traversable(path, turning_radius, wheelbase, max_steering_angle):
    # Check if the path is traversable by the car
    # Implement your traversability check here
    # ...
    for segment_type, segment_length in path:
        if segment_type == "C" or segment_type == "CC":
            # Calculate the minimum turning radius for the current segment
            min_radius = wheelbase / np.tan(max_steering_angle)

            # Check if the given turning radius is smaller than the minimum radius
            if turning_radius < min_radius:
                return False
        elif segment_type == "L" or segment_type == "R":
            # Calculate the maximum steering angle for the current segment
            max_segment_angle = np.arcsin(wheelbase / (2 * turning_radius))

            # Check if the given turning radius exceeds the maximum angle
            if max_steering_angle < max_segment_angle:
                return False
        else:
            # Invalid segment type
            raise ValueError("Invalid segment type in Dubins path")

    return True

# Generate Dubins Path
dubins_path = dubins_path(waypoints[0], waypoints[-1], turning_radius)

# Check traversability
if is_traversable(dubins_path, turning_radius, wheelbase, max_steering_angle):
    print("Path is traversable by the car.")
else:
    print("Path is not traversable by the car.")

dubins_path = np.array(dubins_path)
print(dubins_path)

# Define Dubins maneuvers (replace with your own sequence)
dubins_sequence = [('R', 10)]

# Initialize variables to store the path coordinates
path_x = []
path_y = []

turn_radius = turning_radius

# Initialize the starting position and heading
x = 0.0
y = 0.0
theta = 0.0  # Angle in radians, 0 is the initial heading (east)

# Iterate through the Dubins maneuvers and compute the path
for maneuver_type, arc_length in dubins_sequence:
    if maneuver_type == 'S':
        # Straight-line segment
        for _ in range(int(arc_length)):
            x += math.cos(theta)  # Update x coordinate
            y += math.sin(theta)  # Update y coordinate
            path_x.append(x)
            path_y.append(y)
    else:
        # Arc segment (Left or Right)
        direction = 1 if maneuver_type == 'R' else -1  # Right is positive, Left is negative
        radius = abs(turn_radius)  # Ensure radius is positive
        
        # Calculate the number of steps based on the arc length
        num_steps = int(arc_length * abs(turn_radius))
        for _ in range(num_steps):
            x += radius * math.cos(theta)  # Update x coordinate
            y += radius * math.sin(theta)  # Update y coordinate
            theta += direction * (1 / abs(turn_radius))  # Update heading
            path_x.append(x)
            path_y.append(y)


# Visualization (plot the Dubins path and waypoints)
plt.figure()
plt.plot(path_x, path_y, label="Dubins Path", color='blue')
#plt.plot(dubins_path[:, 0], dubins_path[:, 1], label="Dubins P", color='blue')
plt.scatter(waypoints[:, 0], waypoints[:, 1], label="Waypoints", color='red')
plt.legend()
plt.xlabel('X')
plt.ylabel('Y')
plt.grid()
plt.show()