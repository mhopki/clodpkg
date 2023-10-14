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

def new_node(near_node, rand_node, epsilon, radius, heading):
    dist = distance(near_node, rand_node)
    if dist <= epsilon:
        return rand_node
    theta = math.atan2(rand_node.y - near_node.y, rand_node.x - near_node.x)
    n_theta = theta
    heading_error = theta - heading
    if abs(theta - heading) > radius:
        heading_error = theta - heading
        if heading_error > math.pi:
            heading_error -= 2 * math.pi
        elif heading_error < -math.pi:
            heading_error -= 2 * math.pi
        n_theta = heading + (heading_error/abs(heading_error) * (radius - 0.02))

    #print(theta, " vs ", n_theta)
    if (heading_error * n_theta < 0):
        xx = 0
        #print("AYOOOO", heading_error, n_theta)
    new_x = near_node.x + epsilon * math.cos(n_theta)
    new_y = near_node.y + epsilon * math.sin(n_theta)
    return Node(new_x, new_y)

def rrt_star(start, goal, obstacles, max_iter, epsilon, goal_threshold, radius, heading):
    nodes = [start]
    #print("start: ", start.x, start.y)
    t_heading = heading
    for _ in range(max_iter):
        if radius < 4:
            rand_node = goal
        else:
            rand_node = Node(random.uniform(X_MIN, X_MAX), random.uniform(Y_MIN, Y_MAX))
        nearest_node = min(nodes, key=lambda node: distance(node, rand_node))
        new_node_ = new_node(nearest_node, rand_node, epsilon, radius, t_heading)
        #if nearest_node.parent:
        #    t_heading = calculate_angle([nearest_node.parent.x, nearest_node.parent.y], [nearest_node.x, nearest_node.y])
        #else:
        #    t_heading = heading
        if not collides(nearest_node, new_node_, obstacles) and is_turnable([nearest_node.x, nearest_node.y], [new_node_.x, new_node_.y], radius, t_heading):
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
            #print("t_heading(old): ", t_heading)
            rewire(nodes, new_node_, near_nodes, min_cost_node, epsilon, radius, t_heading)
            n_theta = calculate_angle([new_node_.parent.x, new_node_.parent.y], [new_node_.x, new_node_.y])
            #n_theta = calculate_angle([new_node_.x, new_node_.y], [new_node_.parent.x, new_node_.parent.y])
            if abs(t_heading - n_theta) > radius:
                xxx = 0
                #print("TOO BIG!")
            t_heading = n_theta
            #print("t_heading(new): ", t_heading, " from: ", ([new_node_.parent.x, new_node_.parent.y], [new_node_.x, new_node_.y]))
            """
            if (t_heading >= math.pi):
                t_heading -= 2 * math.pi
            elif (t_heading <= -math.pi):
                t_heading += 2 * math.pi
            """
            #print(t_heading, n_theta)

            if distance(new_node_, goal) < goal_threshold and new_node_.parent is not None and is_turnable([new_node_.x, new_node_.y], [goal.x, goal.y], radius, t_heading):
                print("GOAL")
                final_node = Node(goal.x, goal.y)
                final_node.parent = new_node_
                #print("t_heading(old): ", t_heading)
                n_theta = calculate_angle([final_node.parent.x, final_node.parent.y], [final_node.x, final_node.y])
                #n_theta = calculate_angle([new_node_.x, new_node_.y], [new_node_.parent.x, new_node_.parent.y])
                if abs(t_heading - n_theta) > radius:
                    xxx = 0
                    #print("TOO BIG!")
                t_heading = n_theta
                #print("t_heading(new): ", t_heading, " from: ", ([final_node.parent.x, final_node.parent.y], [final_node.x, final_node.y]))
                return construct_path(final_node)
    
    print(" ")
    print("END ITER")
    print(" ")
    return construct_path(nodes[-1])

def calculate_angle(point1, point2):
    # Calculate the angle (theta) between two points
    delta_x = point2[0] - point1[0]
    delta_y = point2[1] - point1[1]
    
    angle = math.atan2(delta_y, delta_x)
    #print("angle: ", angle)
    
    # Convert angle to degrees if needed
    angle_degrees = math.degrees(angle)
    #print("angle_deg: ", angle_degrees)
    
    return angle

def is_turnable(point1, point2, radius, heading):
    check = calculate_angle(point1, point2)
    if abs(check - heading) <= radius:
        #print(point1, " to ", point2, ": ")
        #print(check, " - ", heading, " < ", radius)
        return True
    else:
        #print("not turnable")
        return False


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

def rewire(nodes, new_node_, near_nodes, min_cost_node, epsilon, radius, heading):
    t_heading = heading
    for node in near_nodes:
        cost_through_new_node = node_cost(nodes, node) + distance(node, new_node_)
        if cost_through_new_node < node_cost(nodes, new_node_):
            if new_node_.parent:
                t_heading = calculate_angle([new_node_.parent.x, new_node_.parent.y], [new_node_.x, new_node_.y])
            else:
                t_heading = heading
            #print("rewire heading: ", t_heading)
            if not collides(node, new_node_, obstacles) and is_turnable([new_node_.x, new_node_.y], [node.x, node.y], radius, t_heading):
                node.parent = new_node_
                #print("rewired!")
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
    'x_min': 3,
    'x_max': 4,
    'y_min': 0,
    'y_max': 3,
    'edges': [
        (Node(3, 0), Node(3, 3)),  # Top edge
        (Node(3, 3), Node(4, 3)),  # Top edge
        (Node(4, 3), Node(4, 0)),  # Top edge
        (Node(4, 0), Node(3, 0))  # Top edge
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
max_iter = 5000 #5000
epsilon = 2.0
goal_threshold = epsilon
radius = 10.0
heading = 0

# Run RRT* algorithm
#path = rrt_star(start, goal, obstacles, max_iter, epsilon, goal_threshold, radius, heading)

path_list = []
print("finished path: ")
# Visualization
plt.figure()
"""for node in path:
    if node.parent:
        if not is_turnable([node.parent.x,node.parent.y],[node.x,node.y], radius, heading):
            xx = 0 
            #print("not turnable")
        path_list.append((node.x, node.y))
        plt.plot([node.x, node.parent.x], [node.y, node.parent.y], 'b-')  # Blue for start and goal nodes"""

true_path = []
max_iter = 200000
epsilon = 0.05
goal_threshold = epsilon
radius = 0.08
heading = 0

decrem = 0.2
obstacles = []
obstacle = {
    'x_min': 3 + decrem,
    'x_max': 4 - decrem,
    'y_min': 0 + decrem,
    'y_max': 3 - decrem,
    'edges': [
        (Node(3 + decrem, 0 + decrem), Node(3 + decrem, 3 - decrem)),  # Top edge
        (Node(3 + decrem, 3 - decrem), Node(4 - decrem, 3 - decrem)),  # Top edge
        (Node(4 - decrem, 3 - decrem), Node(4 - decrem, 0 + decrem)),  # Top edge
        (Node(4 - decrem, 0 + decrem), Node(3 + decrem, 0 + decrem))  # Top edge
    ]
}
obstacles.append(obstacle)
obstacle = {
    'x_min': 0.5 + decrem,
    'x_max': 1.5 - decrem,
    'y_min': 1 + decrem,
    'y_max': 7 - decrem,
    'edges': [
        (Node(0.5 + decrem, 1 + decrem), Node(0.5 + decrem, 7 - decrem)),  # Top edge
        (Node(0.5 + decrem, 7 - decrem), Node(1.5 - decrem, 7 - decrem)),  # Top edge
        (Node(1.5 - decrem, 7 - decrem), Node(1.5 - decrem, 1 + decrem)),  # Top edge
        (Node(1.5 - decrem, 1 + decrem), Node(0.5 + decrem, 1 + decrem))  # Top edge
    ]
}
obstacles.append(obstacle)

path_good = True

"""
for i in range(len(path) - 1):
    #print("new segment: ")
    m_start = Node(path[i].x, path[i].y)
    m_goal = Node(path[i+1].x, path[i+1].y)
    #print("start: ", m_start.x, m_start.y)
    if len(true_path) > 1:
        for i in range(len(true_path) - 1):
            if not (true_path[-(i + 1)].x == m_start.x and true_path[-(i + 1)].y == m_start.y):
                heading = calculate_angle([true_path[-(i + 1)].x, true_path[-(i + 1)].y], [m_start.x, m_start.y])
                #print("tier", (i + 1), " heading: ", heading, " from: ", ([true_path[-(i + 1)].x, true_path[-(i + 1)].y], [m_start.x, m_start.y]))
                break
                #[m_start.x, m_start.y]))
    m_path = []
    m_path = rrt_star(m_start, m_goal, obstacles, max_iter, epsilon, goal_threshold, radius, heading)
    for j in range(len(m_path) - 1):
        #print("m_path: ", m_path[j].x, m_path[j].y)
        if (j == 0):
            if (i == 0):
                true_path.append(m_path[j])
            else:
                xxx = 0
                print("skip first")
        else:
            true_path.append(m_path[j])
    if (not (m_path[-1].x == m_goal.x and m_path[-1].y == m_goal.y)):
        print("PATH FAILED")
        path_good = False
        break
        #if i < len(path)-1:
        #    true_path.pop()"""

#print("finished true path:")
"""
for node in true_path:
    if node.parent:
        print("next node: ")
        is_turnable([node.parent.x,node.parent.y],[node.x,node.y], radius, heading)
        plt.plot([node.x, node.parent.x], [node.y, node.parent.y], 'g-')
"""

"""
c_head = 0
for i in range(len(true_path) - 1):
    heading = calculate_angle([true_path[i].x, true_path[i].y], [true_path[i+1].x, true_path[i+1].y])
    #print("heading: ", heading, "pts: ", ([true_path[i].x, true_path[i].y], [true_path[i+1].x, true_path[i+1].y]))
    if abs(c_head - heading) > radius:
        xxx = 0
        print("TOO BIG")
        path_good = False
        print("heading: ", heading, "pts: ", ([true_path[i].x, true_path[i].y], [true_path[i+1].x, true_path[i+1].y]))
    c_head = heading
    #if not is_turnable([true_path[i].x, true_path[i].y], [true_path[i+1].x, true_path[i+1].y], radius, heading):
    #    print("not turnable")
    #else:
    #    print("heading: ", calculate_angle([true_path[i].x, true_path[i].y], [true_path[i+1].x, true_path[i+1].y]))
    plt.plot([true_path[i+1].x, true_path[i].x], [true_path[i+1].y, true_path[i].y], 'g-')"""

#smoothed_path = smooth_path(path)

#n_points = int(epsilon*len(path)*2)
#n_points = len(path) * 10
#n_points = 100
#print(n_points)
#smoothed_path = smooth_and_curve_path(path, n_points)



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
    xx = 0
    #plt.plot([smoothed_path[i].x, smoothed_path[i + 1].x], [smoothed_path[i].y, smoothed_path[i + 1].y], 'y-')

#rad_path = rad_curve_path(path)

o_off_x = 0
o_off_y = 0

#true_path
if (path_good == True):
    way_path = []
    way_out = PoseStamped()
    way_out.pose.position.x = true_path[0].x + o_off_x
    way_out.pose.position.y = true_path[0].y + o_off_y
    way_pub.publish(way_out)
    way_path.append(Node(true_path[0].x, true_path[0].y))

    """
    last_x = true_path[0].x + o_off_x
    last_y = true_path[0].y + o_off_y
    for i in range(1,len(true_path) - 1):
        if (math.sqrt(((true_path[i].x + o_off_x) - last_x)**2 + ((true_path[i].y + o_off_y) - last_y)**2) >= 0.1):
            way_out = PoseStamped()
            way_out.pose.position.x = true_path[i].x + o_off_x
            way_out.pose.position.y = true_path[i].y + o_off_y
            way_pub.publish(way_out)
            way_path.append(Node(true_path[i].x, true_path[i].y))
            #plt.plot([last_x, smoothed_path[i].x - 2], [last_y, smoothed_path[i].y + 2], 'y-')
            last_x = true_path[i].x + o_off_x
            last_y = true_path[i].y + o_off_y"""

    way_out = PoseStamped()
    way_out.pose.position.x = true_path[-1].x + o_off_x
    way_out.pose.position.y = true_path[-1].y + o_off_y
    way_pub.publish(way_out)
    way_path.append(Node(true_path[-1].x, true_path[-1].y))
    #plt.plot([last_x, smoothed_path[i].x - 2], [last_y, smoothed_path[i].y + 2], 'y-')
    last_x = true_path[i].x + o_off_x
    last_y = true_path[i].y + o_off_y

    for node in way_path:
        xx = 0
        #print("node: ", node.x, node.y)

    for i in range(len(way_path) - 1):
        xxx = 0
        plt.plot([way_path[i].x, way_path[i + 1].x], [way_path[i].y, way_path[i + 1].y], 'r-')

#wayp smooth
if (False):
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
        xx = 0
        #print("node: ", node.x, node.y)

    for i in range(len(way_path) - 1):
        xxx = 0
        #plt.plot([way_path[i].x, way_path[i + 1].x], [way_path[i].y, way_path[i + 1].y], 'r-')

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

#print("start: ", start.x, start.y)
#for node in path:
#    print("node: ", node.x, node.y)
#print("goal: ", goal.x, goal.y)