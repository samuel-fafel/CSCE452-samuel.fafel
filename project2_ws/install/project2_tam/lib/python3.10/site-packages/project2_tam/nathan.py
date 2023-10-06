class Turtle:
     def __init__(self, name="oogway", id=0, segments=[]):
          self.name = name
          self.id = id
          self.segments = segments

def distribute_segments(segments, num_turtles, turtle, workload): #turtle is of class Turtle
	#This relies on segments removing segments that are already distributed
	#workload = len(segments) // num_turtles (needs to be calculated outside of function)
	if (turtle.id == num_turtles):
		for i in range(len(segments)):
			turtle.segments.append(segments.pop(i))
	else:
		for i in range(workload):
			turtle.segments.append(segments.pop(i))


###PID CONTROL FUNCTIONS, MAY OR MAY NOT USE###

def find_angle_error(self, target_x, target_y):
    # Calculate the angle to rotate towards the target point
    angle_to_target = atan2(target_y - self.turtle_pose.y, target_x - self.turtle_pose.x)
    angle_diff = angle_to_target - self.turtle_pose.theta

    # Adjust the angle difference to be within the range [-pi, pi]
    if angle_diff > 3.14159265359:
        angle_diff -= 2 * 3.14159265359
    elif angle_diff < -3.14159265359:
        angle_diff += 2 * 3.14159265359

    return angle_diff


def pid_control(errors, present_const, past_const, future_const, time_dif=1):
     #Define pid_control equation, errors is a list of accumulated errors
    if (len(errors) > 1):
        action = present_const*errors[-1] + past_const*(time_dif*sum(errors)) + future_const*((errors[-1] - errors[-2]) / time_dif)
    else:
        action = present_const*errors[-1] + past_const*(time_dif*sum(errors)) + future_const*(errors[-1] / time_dif) 
    
    return action

#Incomplete
def send_cmdvel_msgs(segment, errors, cmdvel_msg):
    cur_error = find_angle_error(segment.end.x, segment.end.y)
    errors.append(cur_error)

    cmdvel_msg.linear.x = 0
    cmdvel_msg.angular.z = 0

    if (abs(cur_error) < 0.1):
        if (turtle.pose.x == segment.end.x and turtle.pose.y == segment.end.y):
            cmdvel_msg.linear.x = 0
        else:
            cmdvel_msg.linear.x = 1
    else:
        cmdvel_msg.angular.z = pid_control(errors, 1, 1, 0.5, 0.1) #constants will need to be adjusted