import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

# Constants for image and turtle parameters
IMAGE_PATH = '/home/syed/KRSSG Task 3/Image1.png'
START_POINT = (50, 50)  # (x, y) coordinates of the start point
TARGET_POINT = (600, 400)  # (x, y) coordinates of the target point
TURTLE_RADIUS = 0.25  # Radius of the circular turtle motion
THRESHOLD = 2  # Distance threshold for reaching the next point in the path

# Load the minefield image using OpenCV
image = cv2.imread(IMAGE_PATH)

# Preprocess the image to extract black and white regions
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
_, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY_INV)

# Define the RRTConnect class
class RRTConnect:
    def __init__(self, start, goal, obstacles, max_iter=1000, step_size=10):
        self.start = start
        self.goal = goal
        self.obstacles = obstacles
        self.max_iter = max_iter
        self.step_size = step_size

        self.nodes_start = [start]
        self.nodes_goal = [goal]
        self.path = []

    # Function to check if a point is in collision with any obstacles
    def is_collision(self, point):
        for obstacle in self.obstacles:
            if np.linalg.norm(point - obstacle) <= 1:
                return True
        return False

    # Function to find the nearest node to a given point
    def find_nearest_node(self, nodes, point):
        distances = [np.linalg.norm(point - node) for node in nodes]
        min_index = np.argmin(distances)
        return nodes[min_index]

    # Function to steer towards a target point from a given node
    def steer(self, start, target):
        direction = target - start
        norm = np.linalg.norm(direction)
        if norm > self.step_size:
            direction = (direction / norm) * self.step_size
        return start + direction

    # Function to extend the tree towards a random point
    def extend_tree(self, nodes, target):
        nearest = self.find_nearest_node(nodes, target)
        new_node = self.steer(nearest, target)
        if not self.is_collision(new_node):
            nodes.append(new_node)
            return new_node
        return None

    # Function to connect the trees from start to goal
    def connect_trees(self):
        for i in range(self.max_iter):
            random_point = np.random.rand(2) * 100
            extended_node_start = self.extend_tree(self.nodes_start, random_point)
            if extended_node_start:
                extended_node_goal = self.extend_tree(self.nodes_goal, extended_node_start)
                if extended_node_goal:
                    if not self.is_collision(extended_node_goal):
                        self.path = [extended_node_start, extended_node_goal]
                        return

    # Function to find the shortest path between start and goal
    def find_path(self):
        self.connect_trees()
        if len(self.path) == 0:
            return None

        path_start = self.path[0]
        path_goal = self.path[1]
        while path_start not in self.nodes_goal:
            nearest = self.find_nearest_node(self.nodes_goal, path_start)
            extended_node = self.steer(path_start, nearest)
            if not self.is_collision(extended_node):
                self.nodes_goal.append(extended_node)
                path_start = extended_node

        path = []
        current = path_start
        while current != self.start:
            path.append(current)
            nearest = self.find_nearest_node(self.nodes_start, current)
            current = nearest

        path.append(self.start)
        path.reverse()
        path.extend(self.path[1:])
        return path
# Define the PIDController class for turtle movement
class PIDController:
    def __init__(self, constants):
        self.Kp = constants['Kp']
        self.Ki = constants['Ki']
        self.Kd = constants['Kd']
        self.prev_error = 0.0
        self.integral = 0.0

    def calculate(self, error):
        self.integral += error
        derivative = error - self.prev_error
        control_signal = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return control_signal

# Define the pose_callback function
def pose_callback(pose):
    global turtle_pose
    turtle_pose = pose

# Set up the ROS environment and initialize the turtlesim node
rospy.init_node('turtle_control')
cmd_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)
pose_sub = rospy.Subscriber('/turtle1/pose', Pose, pose_callback)

# Variables for turtle control and position tracking
pid_constants = {'Kp': 1.0, 'Ki': 0.0, 'Kd': 0.0}  # PID constants
pid_controller = PIDController(pid_constants)
turtle_pose = None

# Define the callback function for the turtle's position
def pose_callback(pose):
    global turtle_pose
    turtle_pose = pose

# Function to check for collision between turtles
def is_collision():
    global turtle_pose

    turtle_x = turtle_pose.x
    turtle_y = turtle_pose.y

    # Calculate the distance between the two turtles
    distance = math.sqrt((turtle_x - TARGET_POINT[0])**2 + (turtle_y - TARGET_POINT[1])**2)
    return distance <= TURTLE_RADIUS

# Function to update the turtle's movement based on the path
def update_turtle_movement(path):
    global turtle_pose

    # Calculate the distance between the turtle's position and the next point in the path
    distance = math.sqrt((turtle_pose.x - path[0][0])**2 + (turtle_pose.y - path[0][1])**2)

    # Check if the turtle has reached the next point in the path
    if distance <= THRESHOLD:
        # Update the turtle's position to the next point in the path
        turtle_pose.x = path[0][0]
        turtle_pose.y = path[0][1]

        # Remove the reached point from the path
        path.pop(0)

    # Update the image with the turtle's position
    update_image()

    # Calculate the control signal using the PID controller
    control_signal = Twist()
    control_signal.linear.x = 1.0  # Constant forward velocity
    control_signal.angular.z = pid_controller.calculate(0)  # No angular velocity
    cmd_vel_pub.publish(control_signal)

# Function to replan the path if collision is imminent
def replan_path():
    global path, turtle_pose

    # Create a new instance of RRTConnect with the updated start position
    rrt_connect = RRTConnect((turtle_pose.x, turtle_pose.y), TARGET_POINT, [(30, 40), (50, 50), (70, 20)])

    # Find the new path
    path = rrt_connect.find_path()

# Function to update the image with the turtles' positions
def update_image():
    global image, turtle_pose

    # Draw the turtle's position on the image using OpenCV
    turtle_x = int(turtle_pose.x)
    turtle_y = int(turtle_pose.y)
    cv2.circle(image, (turtle_x, turtle_y), 3, (0, 255, 0), -1)

    # Display the updated image
    cv2.imshow('/home/syed/KRSSG Task 3/Image1.png', image)
    cv2.waitKey(1)

# Main function
def main():
    global path

    # Display the initial image
    cv2.imshow('/home/syed/KRSSG Task 3/Image1.png', image)
    cv2.waitKey(0)

    # Initialize the turtle's position to the start point
    turtle_pose = Pose()
    turtle_pose.x = START_POINT[0]
    turtle_pose.y = START_POINT[1]

    # Create an instance of RRTConnect
    rrt_connect = RRTConnect((turtle_pose.x, turtle_pose.y), TARGET_POINT, [(30, 40), (50, 50), (70, 20)])

    # Find the path using RRTConnect
    path = rrt_connect.find_path()
    # Draw the path on the image
    for i in range(len(path) - 1):
        cv2.line(image, (int(path[i][0]), int(path[i][1])), (int(path[i+1][0]), int(path[i+1][1])), (0, 0, 255), 2)


    # Start moving the turtle on the path
    while not rospy.is_shutdown() and turtle_pose is not None:
        # Check for collision and replan the path if necessary
        if is_collision():
            replan_path()

        # Update the turtle's movement based on the path
        update_turtle_movement(path)

        # Update the image with the turtles' positions
        update_image()

        # Publish the control signal as ROS commands to move the turtle
        control_signal = Twist()
        control_signal.linear.x = 1.0  # Constant forward velocity
        control_signal.angular.z = pid_controller.calculate(0)  # No angular velocity
        cmd_vel_pub.publish(control_signal)

    # Close all windows and stop the turtle
    cv2.destroyAllWindows()
    cmd_vel_pub.publish(Twist())

if __name__ == '__main__':
    main()
