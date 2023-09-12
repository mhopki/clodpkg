import matplotlib.pyplot as plt
import random
import math

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

def rrt_star(start, goal, max_iter, epsilon, goal_threshold):
    nodes = [start]
    true_path.append(start)
    for _ in range(max_iter):
        rand_node = Node(random.uniform(X_MIN, X_MAX), random.uniform(Y_MIN, Y_MAX))
        #print("new iter: ", rand_node.x, rand_node.y)
        nearest_node = min(true_path, key=lambda node: distance(node, rand_node))
        #print("nearest: ", nearest_node.x, nearest_node.y)
        new_node_ = new_node(nearest_node, rand_node, epsilon)
        near_nodes = [node for node in true_path if distance(node, new_node_) <= epsilon]
        #for node in near_nodes:
        #    print("nearest_ns: ", node.x, node.y)
        nodes.append(new_node_)
        rewire(nodes, new_node_, near_nodes, epsilon)
        
        # Check if the new node is close enough to the goal
        if distance(new_node_, goal) < goal_threshold and new_node_.parent is not None:
            print("GOAL")
            return construct_path(new_node_)
    
    return construct_path(true_path[-1])  # Return None if goal couldn't be reached

def construct_path(node):
    path = []
    while node is not None:
        path.insert(0, node)
        node = node.parent
    return path

def rewire(nodes, new_node_, near_nodes, epsilon):
    for node in near_nodes:
        cost_through_new_node = node_cost(nodes, node) + distance(node, new_node_)
        #print(cost_through_new_node, " vs ", node_cost(new_node_))
        if cost_through_new_node <= node_cost(nodes, new_node_):
            new_node_.parent = node
            true_path.append(new_node_)
            print("ADDED TO TREE: ", new_node_.x, new_node_.y)
            print("parent: ", node.x, node.y)
        else:
            new_node_.parent = None
            #print("Too far")

def node_cost(nodes, node):
    return distance(node, nodes[0])

# Define the workspace boundaries
X_MIN, X_MAX = 0, 10
Y_MIN, Y_MAX = 0, 10

# Set up the environment
true_path = []
start = Node(1, 1)
goal = Node(9, 9)
goal_threshold = 1.0
max_iter = 50
epsilon = 1.0

# Run RRT* algorithm
path = rrt_star(start, goal, max_iter, epsilon, goal_threshold)

if path:
    print("start: ", start.x, start.y)
    for node in path:
        print("path: ", node.x, node.y)
    print("goal: ", goal.x, goal.y)
else:
    print("No Path")

# Visualization
plt.figure()
for i in range(len(path) - 1):
    plt.plot([path[i].x, path[i + 1].x], [path[i].y, path[i + 1].y], 'b-')
plt.scatter([node.x for node in path], [node.y for node in path], color='blue', label='Path')
plt.scatter(start.x, start.y, color='green', label='Start')
plt.scatter(goal.x, goal.y, color='red', label='Goal')
plt.legend()
plt.xlabel('X')
plt.ylabel('Y')
plt.xlim(X_MIN, X_MAX)
plt.ylim(Y_MIN, Y_MAX)
plt.grid()
plt.show()
