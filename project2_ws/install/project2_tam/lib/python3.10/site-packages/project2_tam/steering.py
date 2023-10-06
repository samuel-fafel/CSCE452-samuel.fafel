from turtlesim.msg import Pose
from math import sqrt, atan2, cos, sin

r = 4

def euclidean_distance(goal, pose):
	"""Euclidean distance between current pose and the goal."""
	return sqrt(pow((goal.x - pose.x), 2) + pow((goal.y - pose.y), 2))

def linear_vel(goal, pose, constant=1):
	return constant * euclidean_distance(goal, pose)

def steering_angle(goal, pose):
	return atan2(goal.y - pose.y, goal.x - pose.x)

def angular_vel(goal, pose, constant=1):
	return constant * (steering_angle(goal, pose) - pose.theta)

def theory_distance(goal, pose):
	new_pose = Pose()
	D = euclidean_distance(goal, pose)
	new_pose.x = pose.x + D*cos(pose.theta)
	new_pose.y = pose.y + D*sin(pose.theta)
	new_pose.theta = pose.theta

	return euclidean_distance(goal, new_pose)