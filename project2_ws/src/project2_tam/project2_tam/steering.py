from math import sqrt, atan2

def euclidean_distance(goal_pose, turtle_pose):
    """Euclidean distance between current pose and the goal."""
    return sqrt(pow((goal_pose.x - turtle_pose.x), 2) +
                pow((goal_pose.y - turtle_pose.y), 2))

def linear_vel(goal_pose, turtle_pose, constant=1.5):
    return constant * euclidean_distance(goal_pose, turtle_pose)

def steering_angle(goal_pose, turtle_pose):
    return atan2(goal_pose.y - turtle_pose.y, goal_pose.x - turtle_pose.x)

def angular_vel(goal_pose, turtle_pose, constant=6):
    return constant * (steering_angle(goal_pose, turtle_pose) - turtle_pose.theta)