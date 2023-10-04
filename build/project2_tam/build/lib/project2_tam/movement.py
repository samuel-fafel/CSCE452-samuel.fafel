def rotate_towards_point(self, target_x, target_y):
        # Calculate the angle to rotate towards the target point
        angle_to_target = atan2(target_y - self.turtle_pose.y, target_x - self.turtle_pose.x)
        angle_diff = angle_to_target - self.turtle_pose.theta

        # Adjust the angle difference to be within the range [-pi, pi]
        if angle_diff > 3.14159265359:
            angle_diff -= 2 * 3.14159265359
        elif angle_diff < -3.14159265359:
            angle_diff += 2 * 3.14159265359

        # Rotate the turtle towards the target point
        while abs(angle_diff) > 0.1:
            twist_msg = Twist()
            twist_msg.angular.z = 0.5 if angle_diff > 0 else -0.5
            self.pub_cmd_vel.publish(twist_msg)
            rclpy.spin_once(self.node)
            angle_diff = angle_to_target - self.turtle_pose.theta