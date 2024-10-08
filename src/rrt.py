#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
import matplotlib.pyplot as plt
import math
from geometry_msgs.msg import PoseStamped

np.random.seed(5)

class RRTPlanner:
    def __init__(self, start, goal, obstacle_list, mapsize=(10,10)):
        self.start = start
        self.goal = goal
        self.obstacle_list = obstacle_list
        self.tree = {tuple(start): (None)}
        self.consec_col = 0
        self.mapsize = mapsize

    def generate_random_point(self):
        return np.random.rand(2) * np.array(self.mapsize)  # Random point in a 10x10 grid (adjust as needed)

    def nearest_neighbor(self, point):
        distances = [np.linalg.norm(np.array(point[:2]) - np.array(node[:2])) for node in self.tree.keys()]
        nearest_node = list(self.tree.keys())[np.argmin(distances)]
        return nearest_node

    def steer(self, from_point, to_point, max_dist, max_angle):
       	direction = np.array(to_point[:2]) - np.array(from_point[:2])
        direction_norm = np.linalg.norm(direction)

        # Initial out_dist as the distance between from_point and to_point
        out_dist = direction_norm

        # If out_dist is larger than max_dist, set it to max_dist
        if out_dist > max_dist:
        	out_dist = max_dist

        # Steer towards the adjusted direction
        new_point = tuple(np.array(from_point[:2]) + out_dist * direction / direction_norm)

        return new_point
	    

    def collision_free(self, from_point, to_point):
	    # Line segment parameters
	    x1, y1 = from_point[:2]
	    x2, y2 = to_point[:2]

	    # Check intersection with each obstacle
	    for obstacle in self.obstacle_list:
	        # Rectangle vertices
	        x3, y3, x4, y4 = obstacle

	        # Check if the line segment intersects with any of the obstacle edges
	        #if self.do_line_segments_intersect(x1, y1, x2, y2, x3, y3, x4, y4):
	        if self.line_intersects_line([x1, y1], [x2, y2], [x3, y3], [x4, y4]):
	            #print("NO GOOD :()", obstacle)
	            return False  # Collision detected

	    #print("GOOD!")
	    return True  # No collision detected

    def line_intersects_rectangle(self, x1, y1, x2, y2, x3, y3, x4, y4):
	    # Check if the line segment intersects with the obstacle's bounding box
	    x_min = min(x3, x4)
	    x_max = max(x3, x4)
	    y_min = min(y3, y4)
	    y_max = max(y3, y4)

	    o_x_min = min(x1, x2)
	    o_x_max = max(x1, x2)
	    o_y_min = min(y1, y2)
	    o_y_max = max(y1, y2)
	    
	    if (x_max < o_x_min or x_min > o_x_max or
	        y_max < o_y_min or y_min > o_y_max):
	        return False
	    
	    return True

    def point_intersects_rectangle(self, x1, y1, x2, y2, x3, y3, x4, y4):
	    # Check if the line segment intersects with the obstacle's bounding bo

	    o_x_min = min(x3, x4)
	    o_x_max = max(x3, x4)
	    o_y_min = min(y3, y4)
	    o_y_max = max(y3, y4)
	    
	    if (x2 > o_x_min and x2 < o_x_max and y2 > o_y_min and y2 < o_y_max):
	        return True
	    
	    return False

    """
    def do_line_segments_intersect(self, x1, y1, x2, y2, x3, y3, x4, y4):
	    # Determine orientation of the line segments
	    def orientation(p1, p2, p3):
	        return (p2[1] - p1[1]) * (p3[0] - p2[0]) - (p2[0] - p1[0]) * (p3[1] - p2[1])

	    o1 = orientation((x1, y1), (x2, y2), (x3, y3))
	    o2 = orientation((x1, y1), (x2, y2), (x4, y4))
	    o3 = orientation((x3, y3), (x4, y4), (x1, y1))
	    o4 = orientation((x3, y3), (x4, y4), (x2, y2))

	    # General case for intersection
	    if (o1 != o2) and (o3 != o4):
	        return True

	    # Special cases for collinear points and endpoints of one segment lying on the other
	    if (o1 == 0) and on_segment((x1, y1), (x3, y3), (x2, y2)):
	        return True
	    if (o2 == 0) and on_segment((x1, y1), (x4, y4), (x2, y2)):
	        return True
	    if (o3 == 0) and on_segment((x3, y3), (x1, y1), (x4, y4)):
	        return True
	    if (o4 == 0) and on_segment((x3, y3), (x2, y2), (x4, y4)):
	        return True

	    return False

    def on_segment(self, p, q, r):
	    if (q[0] <= max(p[0], r[0])) and (q[0] >= min(p[0], r[0])) and (q[1] <= max(p[1], r[1])) and (q[1] >= min(p[1], r[1])):
	        return True
	    return False
	"""
    def line_intersects_line(self, node1, node2, edge_start, edge_end):
	    def orientation(p, q, r):
	        val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
	        if val == 0:
	            return 0
	        return 1 if val > 0 else 2

	    o1 = orientation(node1, node2, edge_start)
	    o2 = orientation(node1, node2, edge_end)
	    o3 = orientation(edge_start, edge_end, node1)
	    o4 = orientation(edge_start, edge_end, node2)

	    if o1 != o2 and o3 != o4:
	        return True

	    if (o1 == 0 and self.is_on_segment(node1, edge_start, node2)) or \
	       (o2 == 0 and self.is_on_segment(node1, edge_end, node2)) or \
	       (o3 == 0 and self.is_on_segment(edge_start, node1, edge_end)) or \
	       (o4 == 0 and self.is_on_segment(edge_start, node2, edge_end)):
	        return True

	    return False

    def is_on_segment(self, p, q, r):
	    return (q[0] <= max(p[0], r[0]) and q[0] >= min(p[0], r[0]) and
	            q[1] <= max(p[1], r[1]) and q[1] >= min(p[1], r[1]))




    def point_distance(self, point1, point2):
    	x1, y1 = point1
    	x2, y2 = point2

    	return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def point_direction(self, point1, point2):
    	x1, y1 = point1
    	x2, y2 = point2

    	dx = x2 - x1
    	dy = y2 - y1

    	# Calculate the angle between the current direction and the target direction
    	angle = math.atan2(dy, dx)

    	return angle

    def find_distance(self, point1, point2):
    	return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def plan(self, max_iter=10000, max_dist=20.0, min_dist=2.0, max_ang=6.24):#10000 0.8 0.2 0.35
        for _ in range(max_iter):
            random_point = self.generate_random_point()
            nearest_node = self.nearest_neighbor(random_point)
            node_dist = self.point_distance(np.array(nearest_node[:2]), random_point)
            node_ang = self.point_direction(np.array(nearest_node[:2]), random_point)
            min_ang = max_ang
            true_max_ang = 2*math.pi
            max_dist_aug = 3.0
            ang_scaled = min_ang + (true_max_ang - min_ang) * (node_dist - min_dist) / (max_dist_aug - min_dist)#max_ang #(max_ang/2) * ((node_dist / max_dist)) + (max_ang/2)
            #print(ang_scaled, node_dist)

            """
            while node_dist < min_dist or abs(node_ang - nearest_node[2]) > ang_scaled:
            	#print("NOPE", abs(node_ang - nearest_node[2]))
            	random_point = self.generate_random_point()
            	nearest_node = self.nearest_neighbor(random_point)
            	node_dist = self.point_distance(np.array(nearest_node[:2]), random_point)
            	node_ang = self.point_direction(np.array(nearest_node[:2]), random_point)
            """
            if (node_dist < min_dist or abs(node_ang - nearest_node[2]) > ang_scaled):
            	#print("cont")
            	continue
            #print("YES", random_point)

            new_point = self.steer(nearest_node, random_point, max_dist, max_ang)
            if self.collision_free(nearest_node, new_point):
                self.consec_col = 0
                point_out = (new_point[0], new_point[1], node_ang)
                self.tree[tuple(point_out)] = nearest_node
                node_dist = self.point_distance(np.array(point_out[:2]), self.goal[:2])
                node_ang = self.point_direction(np.array(point_out[:2]), self.goal[:2])
                if node_dist < max_dist and abs(node_ang - point_out[2]) < ang_scaled:
                	nearest_node = point_out
	                #node_ang = self.point_direction(np.array(nearest_node[:2]), self.goal[:2])
	                goal_point = self.steer(nearest_node, self.goal, max_dist, max_ang)
                	if self.collision_free(nearest_node, goal_point):
	                    self.tree[tuple(goal_point)] = (point_out)
	                    #print("Path found!")
	                    return self.construct_path(goal_point)
                	else:
	                    self.consec_col += 1
            else:
	            self.consec_col += 1

            #print("Collisions: ", self.consec_col)
        print("Could not find path after max iterations.")
        print(self.tree)
        return None

    def construct_path(self, end_point):
        path = []
        current_node = end_point
        while current_node is not None:
            path.append(current_node)
            current_node = self.tree[current_node]
        return path

def plot_obstacles(obstacle_list):
    plt.gca().set_aspect('equal', adjustable='box')
    for obstacle in obstacle_list:
        x1, y1, x2, y2 = obstacle
        plt.plot([x1, x2], [y1, y1], 'r')
        plt.plot([x2, x2], [y1, y2], 'r')
        plt.plot([x2, x1], [y2, y2], 'r')
        plt.plot([x1, x1], [y2, y1], 'r')

def plot_path(path):
    plt.gca().set_aspect('equal', adjustable='box')
    x_vals = [point[0] for point in path]
    y_vals = [point[1] for point in path]
    plt.plot(x_vals, y_vals, '-bo')

def send_waypoints(path, way_pub):
	for i in range(len(path)):
		max_i = len(path) - 1
		way_out = PoseStamped()
		way_out.pose.position.x = path[max_i - i][0]
		way_out.pose.position.y = path[max_i - i][1]
		way_pub.publish(way_out)
		print(path[max_i - i])

def main():
    rospy.init_node('rrt_planner', anonymous=True)
    way_pub = rospy.Publisher('/waypoints', PoseStamped, queue_size=100)

    #CHANGE THESE TO CHANGE THE OUTPUT
    start = (0,0,0)#(0,0,0)#(9, 1, 3.14)  # Define start point
    goal = (2, 0, 0)#((1.4),(10),0)#((1.4*2*3),(10*2*1.5),0)#(3,10,0)#(6,25,0)#(3,5,0) #(10, 25, 0)   # Define goal point

    #PATH MUST FIT WITHIN THE MAPSIZE (start, goal, and inbetween)
    mapsize = (100,100)#(7,4)#(15,40)#(40,15)#(7,4)#(40,15)

    # Define the 10x10 array of obstacles (0s and 1s)
    obstacle_array = np.zeros(mapsize)
    obstacle_array = [
	    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	  0,0,0,0,0,0],
	 [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	  0,0,0,0,0,0],
	 [0,0,1,1,1,0,0,0,0,1,1,1,0,0,0,0,1,1,1,0,0,0,0,1,1,1,0,0,0,0,1,1,1,0,0,0,
	  0,1,1,1,0,0],
	 [0,0,1,1,1,0,0,0,0,1,1,1,0,0,0,0,1,1,1,0,0,0,0,1,1,1,0,0,0,0,1,1,1,0,0,0,
	  0,1,1,1,0,0],
	 [0,0,1,1,1,0,0,0,0,1,1,1,0,0,0,0,1,1,1,0,0,0,0,1,1,1,0,0,0,0,1,1,1,0,0,0,
	  0,1,1,1,0,0],
	 [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	  0,0,0,0,0,0],
	 [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	  0,0,0,0,0,0],
	 [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	  0,0,0,0,0,0],
	 [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	  0,0,0,0,0,0],
	 [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	  0,0,0,0,0,0],
	 [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	  0,0,0,0,0,0],
	 [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	  0,0,0,0,0,0],
	 [0,0,1,1,1,0,0,0,0,1,1,1,0,0,0,0,1,1,1,0,0,0,0,1,1,1,0,0,0,0,1,1,1,0,0,0,
	  0,1,1,1,0,0],
	 [0,0,1,1,1,0,0,0,0,1,1,1,0,0,0,0,1,1,1,0,0,0,0,1,1,1,0,0,0,0,1,1,1,0,0,0,
	  0,1,1,1,0,0],
	 [0,0,1,1,1,0,0,0,0,1,1,1,0,0,0,0,1,1,1,0,0,0,0,1,1,1,0,0,0,0,1,1,1,0,0,0,
	  0,1,1,1,0,0],
	 [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	  0,0,0,0,0,0],
	 [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	  0,0,0,0,0,0],
	 [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	  0,0,0,0,0,0]
    ]
    """
    obstacle_array = [
    	[0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1],
    	[1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0]

    ]"""
    """[
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0, 0, 1, 1, 1],
        [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 1, 1, 0, 0, 0, 0, 0, 0]
    ]"""

    """
    obstacle_array = [
    	[0, 0, 0, 0, 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0],
    	[0, 0, 0, 0, 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0],
    	[0, 0, 0, 0, 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0],
    	[0, 0, 0, 0, 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0],
    	[0, 0, 0, 0, 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0],
    	[0, 0, 0, 0, 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0],
    	[0, 0, 0, 0, 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0],
    	[0, 0, 0, 0, 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0],
    	[0, 0, 0, 0, 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0],
    	[0, 0, 0, 0, 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0],
    	[0, 0, 0, 0, 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0],
    	[0, 0, 0, 0, 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0],
    	[0, 0, 0, 0, 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0],
    	[0, 0, 0, 0, 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0],
    	[0, 0, 0, 0, 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0],
    	[0, 0, 0, 0, 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0],
    	[0, 0, 0, 0, 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0],
    	[0, 0, 0, 0, 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0],
    	[0, 0, 0, 0, 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0],
    	[0, 0, 0, 0, 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0],
    	[0, 0, 0, 0, 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0],
    	[0, 0, 0, 0, 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0],
    	[0, 0, 0, 0, 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0],
    	[0, 0, 0, 0, 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0],
    	[0, 0, 0, 0, 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0],
 		[0, 0, 0, 0, 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0],
 		[0, 0, 0, 0, 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0],
 		[0, 1, 0, 0, 0 0 0 0 1 0 0 0 0 0 0 1 0 0 0 0 0 0 1 0 0 0 0 0 0 1 0 0 0 0 0 0 1 0 0],
 		[0, 0, 0, 0, 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0],
		[0, 0, 0, 0, 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0],
		[0, 0, 0, 0, 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0],
		[0, 0, 0, 0, 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0],
 		[0, 0, 0, 0, 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0],
 		[0, 0, 0, 0, 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0],
 		[0, 0, 0, 0, 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0],
 		[0, 0, 0, 0, 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0],
 		[0, 0, 0, 0, 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0],
		[0, 1, 0, 0, 0 0 0 0 1 0 0 0 0 0 0 1 0 0 0 0 0 0 1 0 0 0 0 0 0 1 0 0 0 0 0 0 1 0 0],
		[0, 0, 0, 0, 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0],
		[0, 0, 0, 0,, 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
	]"""

	#EMPTY OBSTACLE ARRAY AND LIST!!!
    obstacle_array = []

    obstacle_list = []

    # Convert the obstacle array into obstacle coordinates
    for i in range(len(obstacle_array)):
        for j in range(len(obstacle_array[0])):
            if obstacle_array[i][j] == 1:
                y1 = j - 1
                x1 = len(obstacle_array) - i - 1  # Invert y-axis
                y2 = y1 + 1
                x2 = x1 + 1
                obstacle_list.append((x1 * 1 - 2, y1 * 1 -12, x2 * 1 - 2, y2 * 1 - 12))

    #obstacle_list.append((3.5, 4.25, 5.6, 5.75))  # Define obstacle(s) as (x1, y1, x2, y2)
    #obstacle_list.append((1.5 - 0.5, 3 - 0.5, 1.5 + 0.5, 3 + 0.5))

    # Plot obstacles and path
    plt.figure()
    plot_obstacles(obstacle_list)
    plt.plot(start[0], start[1], 'go')  # Plot start point
    plt.plot(goal[0], goal[1], 'yo')    # Plot goal point
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('RRT Path Planning')
    plt.grid(True)
    #plt.show()

    planner = None
    
    max_attempts = 20  # Maximum number of attempts to find a path
    for attempt in range(1, max_attempts + 1):
        print(f"Attempt {attempt}/{max_attempts}")
	    
        planner = RRTPlanner(start, goal, obstacle_list, mapsize)
        path = planner.plan()

        if path:
            # Path found, break out of the loop
            print("Path found!")
            break
        else:
            # No path found, try again
            print("No path found. Retrying...")

    if path:

        #print("Path:", path[::-1])
        send_waypoints(path, way_pub)
        
        # Plot obstacles and path
        plt.figure()
        plot_obstacles(obstacle_list)
        plot_path(path)
        plt.plot(start[0], start[1], 'go')  # Plot start point
        plt.plot(goal[0], goal[1], 'yo')    # Plot goal point
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('RRT Path Planning')
        plt.grid(True)
        plt.show()

if __name__ == '__main__':
    main()
