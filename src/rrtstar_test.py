#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

import matplotlib.pyplot as plt
import random
import math
import numpy as np
from scipy.interpolate import CubicSpline

from scipy.optimize import minimize

# Define the workspace boundaries
X_MIN, X_MAX = 0, 4
Y_MIN, Y_MAX = 0, 7

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

def distance(node1, node2):
    return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)

def new_node(near_node, rand_node, epsilon):
    dist = distance(near_node, rand_node)
    if dist <= epsilon:
        return rand_node
    theta = math.atan2(rand_node.y - near_node.y, rand_node.x - near_node.x)
    new_x = near_node.x + epsilon * math.cos(theta)
    new_y = near_node.y + epsilon * math.sin(theta)
    return Node(new_x, new_y)

def rrt_star(start, goal, obstacles, max_iter, epsilon, goal_threshold):
    nodes = [start]
    for _ in range(max_iter):
        rand_node = Node(random.uniform(X_MIN, X_MAX), random.uniform(Y_MIN, Y_MAX))
        nearest_node = min(nodes, key=lambda node: distance(node, rand_node))
        new_node_ = new_node(nearest_node, rand_node, epsilon)
        if not collides(nearest_node, new_node_, obstacles):
            near_nodes = [node for node in nodes if distance(node, new_node_) <= epsilon]
            min_cost_node = nearest_node
            min_cost = float('inf')
            for node in near_nodes:
                if not collides(node, new_node_, obstacles):
                    cost = node_cost(nodes, node) + distance(node, new_node_)
                    if cost < min_cost:
                        min_cost_node = node
                        min_cost = cost
            new_node_.parent = min_cost_node
            nodes.append(new_node_)
            rewire(nodes, new_node_, near_nodes, min_cost_node, epsilon)

            if distance(new_node_, goal) < goal_threshold and new_node_.parent is not None:
                print("GOAL")
                final_node = Node(goal.x, goal.y)
                final_node.parent = new_node_
                return construct_path(final_node)
    
    return construct_path(nodes[-1])

def construct_path(node):
    path = []
    while node is not None:
        path.insert(0, node)
        node = node.parent
    return path

def collides(node1, node2, obstacles):
    for obstacle in obstacles:
        if line_intersects_rectangle(node1, node2, obstacle):
            return True
    return False

def line_intersects_rectangle(node1, node2, obstacle):
    # Check if the line segment intersects with the obstacle's bounding box
    x_min = min(node1.x, node2.x)
    x_max = max(node1.x, node2.x)
    y_min = min(node1.y, node2.y)
    y_max = max(node1.y, node2.y)
    
    if (x_max < obstacle['x_min'] or x_min > obstacle['x_max'] or
        y_max < obstacle['y_min'] or y_min > obstacle['y_max']):
        return False
    
    # Check if the line segment intersects with any of the obstacle's edges
    for edge in obstacle['edges']:
        if line_intersects_line(node1, node2, edge[0], edge[1]):
            return True
    
    return False

def line_intersects_line(node1, node2, edge_start, edge_end):
    # Check if the two line segments intersect
    def orientation(p, q, r):
        val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y)
        if val == 0:
            return 0
        return 1 if val > 0 else 2
    
    o1 = orientation(node1, node2, edge_start)
    o2 = orientation(node1, node2, edge_end)
    o3 = orientation(edge_start, edge_end, node1)
    o4 = orientation(edge_start, edge_end, node2)
    
    if o1 != o2 and o3 != o4:
        return True
    
    if (o1 == 0 and is_on_segment(node1, edge_start, node2)) or (o2 == 0 and is_on_segment(node1, edge_end, node2)) or (o3 == 0 and is_on_segment(edge_start, node1, edge_end)) or (o4 == 0 and is_on_segment(edge_start, node2, edge_end)):
        return True
    
    return False

def is_on_segment(p, q, r):
    return (q.x <= max(p.x, r.x) and q.x >= min(p.x, r.x) and
            q.y <= max(p.y, r.y) and q.y >= min(p.y, r.y))


def node_cost(nodes, node):
    # Example: Calculate cost based on distance from the start node
    #start_node = nodes[0]  # Assuming the start node is the first node added
    #return distance(start_node, node) + 1.0
    return distance(node, nodes[0])

def rewire(nodes, new_node_, near_nodes, min_cost_node, epsilon):
    for node in near_nodes:
        cost_through_new_node = node_cost(nodes, node) + distance(node, new_node_)
        if cost_through_new_node < node_cost(nodes, new_node_):
            if not collides(node, new_node_, obstacles):
                node.parent = new_node_
                for existing_node in nodes:
                    if existing_node.parent == node:
                        existing_node.parent = None

def smooth_path(path, weight_data=0.5, weight_smooth=0.1, tolerance=0.00001):
    smoothed_path = path.copy()
    change = tolerance
    while change >= tolerance:
        change = 0
        for i in range(1, len(path) - 1):
            for j in range(2):
                before = smoothed_path[i].x if j == 0 else smoothed_path[i].y
                original = path[i].x if j == 0 else path[i].y
                after = smoothed_path[i].x if j == 0 else smoothed_path[i].y
                
                difference = weight_data * (original - after) + weight_smooth * (before + after - 2 * original)
                smoothed_path[i] = Node(smoothed_path[i].x - (1 - j) * difference, smoothed_path[i].y - j * difference)
                change += abs(difference)
    return smoothed_path


def smooth_and_curve_path(path, num_points=1000):
    x_coords = [node.x for node in path]
    y_coords = [node.y for node in path]

    t = list(range(len(path)))
    spline_x = CubicSpline(t, x_coords)
    spline_y = CubicSpline(t, y_coords)

    t_smooth = np.linspace(0, len(path) - 1, num_points)
    x_smooth = spline_x(t_smooth)
    y_smooth = spline_y(t_smooth)
    #spline = CubicSpline(x_coords, )
    #smooth_curvature = spline(t)

    #t_yaw = np.cumsum(smooth_curvature) * (t[1] - t[0])

    smoothed_and_curved_path = [Node(x, y) for x, y in zip(x_smooth, y_smooth)]
    #smoothed_and_curved_path = [Node(x, y) for x, y in zip(t, t_yaw)]
    return smoothed_and_curved_path


"""
def smooth_and_curve_path(path, turn_radius, num_points=1000):
    # Compute desired curvature values based on the desired turn radius
    desired_curvature = 1 / turn_radius
    des_list = [desired_curvature] * len(path)

    # Assuming path is a list of Node objects with x and y coordinates
    x_coords = [node.x for node in path]
    y_coords = [node.y for node in path]

    # Create a parameterized path
    t = list(range(len(path)))

    # Create splines for x and y based on desired curvature
    spline_curvature = CubicSpline(t, des_list)

    # Compute the curvature values along the path
    t_smooth = np.linspace(0, len(path) - 1, num_points)
    curvature_smooth = spline_curvature(t_smooth)

    # Integrate curvature values to get yaw (angle)
    yaw_smooth = np.cumsum(curvature_smooth) * (t_smooth[1] - t_smooth[0])

    # Compute x and y coordinates based on yaw
    x_smooth = np.cumsum(np.cos(yaw_smooth)) * (x_coords[-1] - x_coords[0]) / len(path)
    y_smooth = np.cumsum(np.sin(yaw_smooth)) * (y_coords[-1] - y_coords[0]) / len(path)

    smoothed_and_curved_path = [Node(x, y) for x, y in zip(x_smooth, y_smooth)]
    return smoothed_and_curved_path
"""

"""
def interpolate_path_segment(segment, num_points):
    # Assuming segment is a list of Nodes
    x_coords = [node.x for node in segment]
    y_coords = [node.y for node in segment]

    t = list(range(len(segment)))
    spline_x = CubicSpline(t, x_coords)
    spline_y = CubicSpline(t, y_coords)

    t_smooth = np.linspace(0, len(segment) - 1, num_points)
    x_smooth = spline_x(t_smooth)
    y_smooth = spline_y(t_smooth)

    return [Node(x, y) for x, y in zip(x_smooth, y_smooth)]

def smooth_and_curve_path(path, num_points=1000):
    # Split the path into segments where x is monotonically increasing or decreasing
    segments = []
    current_segment = [path[0]]
    for i in range(1, len(path)):
        if path[i].x >= path[i - 1].x:
            current_segment.append(path[i])
        else:
            # Zigzag detected, end current segment and start a new one
            segments.append(current_segment)
            current_segment = [path[i]]

    # Add the last segment
    segments.append(current_segment)

    # Interpolate each segment and combine the results
    interpolated_segments = [interpolate_path_segment(segment, num_points) for segment in segments]
    smoothed_and_curved_path = [node for segment in interpolated_segments for node in segment]

    return smoothed_and_curved_path
"""

"""
def rad_curve_path(path, num_points=1000):
    # Example data points
    x = np.array([node.x for node in path])
    y = np.array([node.y for node in path])


    # Example turn radius values (calculated based on orientation)
    turn_radius = []
    while len(turn_radius) < len(path):
        turn_radius.append(float('inf'))
        turn_radius.append(2.0)
    turn_radius = np.array(turn_radius[0:len(path)])
    print(len(x), len(turn_radius))

    # Calculate curvature from turn radius
    curvature = 1.0 / turn_radius

    # Create a cubic spline using curvature as the y-values
    spline = CubicSpline(x, curvature, bc_type='natural')  # bc_type='natural' ensures the endpoints have zero curvature

    # Evaluate the spline at many points for a smooth path
    t = np.linspace(0, len(path) - 1, num_points)
    smooth_curvature = spline(t)

    # Integrate the curvature to get smooth yaw (orientation)
    smooth_yaw = np.cumsum(smooth_curvature) * (t[1] - t[0])

    # Plot the original data points and the smoothed path
    plt.scatter(x, y, label='Data Points')
    plt.plot(t, smooth_yaw, label='Smoothed Yaw', color='red')
    plt.xlabel('Time')
    plt.ylabel('Yaw (Orientation)')
    plt.legend()
    plt.grid()
    plt.show()
"""


# Define a function to calculate curvature
def curvature(x, y):
    dx_dt = np.gradient(x)
    dy_dt = np.gradient(y)
    d2x_dt2 = np.gradient(dx_dt)
    d2y_dt2 = np.gradient(dy_dt)
    return np.abs((dx_dt * d2y_dt2 - dy_dt * d2x_dt2) / ((dx_dt**2 + dy_dt**2)**(3/2)))

# Define a function to minimize curvature
def minimize_curvature(params):
    x, y = params[:len(params)//2], params[len(params)//2:]
    return -np.mean(curvature(x, y))  # Minimize negative curvature


# Set up the environment
rospy.init_node('rrt_cont', anonymous=True)
way_pub = rospy.Publisher('/waypoints', PoseStamped, queue_size=100)
start = Node(0.2, 0.2)
goal = Node(3.4, 6)
obstacles = []
obstacle = {
    'x_min': 2,
    'x_max': 4,
    'y_min': 0,
    'y_max': 3,
    'edges': [
        (Node(2, 0), Node(2, 3)),  # Top edge
        (Node(2, 3), Node(4, 3)),  # Top edge
        (Node(4, 3), Node(4, 0)),  # Top edge
        (Node(4, 0), Node(2, 0))  # Top edge
    ]
}
obstacles.append(obstacle)
obstacle = {
    'x_min': 0.5,
    'x_max': 1.5,
    'y_min': 1,
    'y_max': 7,
    'edges': [
        (Node(0.5, 1), Node(0.5, 7)),  # Top edge
        (Node(0.5, 7), Node(1.5, 7)),  # Top edge
        (Node(1.5, 7), Node(1.5, 1)),  # Top edge
        (Node(1.5, 1), Node(0.5, 1))  # Top edge
    ]
}
obstacles.append(obstacle)
max_iter = 5000
epsilon = 1.0
goal_threshold = epsilon

# Run RRT* algorithm
path = rrt_star(start, goal, obstacles, max_iter, epsilon, goal_threshold)

#smoothed_path = smooth_path(path)

n_points = int(epsilon*len(path)*2)
n_points = len(path) * 10
n_points = 100
print(n_points)
smoothed_path = smooth_and_curve_path(path, n_points)

path_list = []

# Visualization
plt.figure()
for node in path:
    if node.parent:
        path_list.append((node.x, node.y))
        plt.plot([node.x, node.parent.x], [node.y, node.parent.y], 'b-')  # Blue for start and goal nodes

# Plot the obstacle
for obstacle_i in obstacles:
    obstacle_rect = plt.Rectangle((obstacle_i['x_min'] + 0.1, obstacle_i['y_min'] + 0.1),
                                   obstacle_i['x_max'] - obstacle_i['x_min'] - 0.2,
                                   obstacle_i['y_max'] - obstacle_i['y_min'] - 0.2,
                                   color='gray', alpha=0.5)
    plt.gca().add_patch(obstacle_rect)
    for edge in obstacle_i['edges']:
        obst_edges = plt.plot([edge[0].x, edge[1].x], [edge[0].y, edge[1].y], color='gray')

for i in range(len(smoothed_path) - 1):
    plt.plot([smoothed_path[i].x, smoothed_path[i + 1].x], [smoothed_path[i].y, smoothed_path[i + 1].y], 'y-')

#rad_path = rad_curve_path(path)

#wayp smooth
if (True):
    way_path = []
    way_out = PoseStamped()
    way_out.pose.position.x = smoothed_path[0].x - 2
    way_out.pose.position.y = smoothed_path[0].y + 2
    way_pub.publish(way_out)
    way_path.append(Node(smoothed_path[0].x, smoothed_path[0].y))

    last_x = smoothed_path[0].x - 2
    last_y = smoothed_path[0].y + 2
    for i in range(1,len(smoothed_path) - 1):
        if math.sqrt(((smoothed_path[i].x - 2) - last_x)**2 + ((smoothed_path[i].y + 2) - last_y)**2) >= 2.0:
            way_out = PoseStamped()
            way_out.pose.position.x = smoothed_path[i].x - 2
            way_out.pose.position.y = smoothed_path[i].y + 2
            way_pub.publish(way_out)
            way_path.append(Node(smoothed_path[i].x, smoothed_path[i].y))
            #plt.plot([last_x, smoothed_path[i].x - 2], [last_y, smoothed_path[i].y + 2], 'y-')
            last_x = smoothed_path[i].x - 2
            last_y = smoothed_path[i].y + 2

    way_out = PoseStamped()
    way_out.pose.position.x = smoothed_path[-1].x - 2
    way_out.pose.position.y = smoothed_path[-1].y + 2
    way_pub.publish(way_out)
    way_path.append(Node(smoothed_path[-1].x, smoothed_path[-1].y))
    #plt.plot([last_x, smoothed_path[i].x - 2], [last_y, smoothed_path[i].y + 2], 'y-')
    last_x = smoothed_path[i].x - 2
    last_y = smoothed_path[i].y + 2

    for node in way_path:
        print("node: ", node.x, node.y)

    for i in range(len(way_path) - 1):
        xxx = 0
        plt.plot([way_path[i].x, way_path[i + 1].x], [way_path[i].y, way_path[i + 1].y], 'r-')

#wayp rigid
if (False):
    for i in range(0,len(path) - 1):
        way_out = PoseStamped()
        way_out.pose.position.x = path[i].x - 2
        way_out.pose.position.y = path[i].y + 2
        way_pub.publish(way_out)

    way_out = PoseStamped()
    way_out.pose.position.x = smoothed_path[-1].x - 2
    way_out.pose.position.y = smoothed_path[-1].y + 2
    way_pub.publish(way_out)

plt.scatter([node.x for node in path], [node.y for node in path], color='blue', label='Path')
plt.scatter([node.x for node in path], [node.y for node in path], color='yellow', label='Path(Smooth)')
plt.scatter(start.x, start.y, color='green', label='Start')
plt.scatter(goal.x, goal.y, color='red', label='Goal')
plt.scatter(0,0, color='gray', label='Obstacles')
plt.legend()
plt.xlabel('X')
plt.ylabel('Y')
plt.xlim(X_MIN, X_MAX)
plt.ylim(Y_MIN, Y_MAX)
plt.grid()
plt.show()

print("start: ", start.x, start.y)
for node in path:
    print("node: ", node.x, node.y)
print("goal: ", goal.x, goal.y)