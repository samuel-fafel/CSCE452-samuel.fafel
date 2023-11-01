#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sklearn.cluster import DBSCAN
import numpy as np
import math

class Obstacle:
    def __init__(self, obstacle_id, position):
        self.obstacle_id = obstacle_id
        self.position = position  # Position is a 2D tuple (x, y)
        self.person = False

    def update_position(self, new_position):
        self.position = new_position

class PeopleTracker(Node):
    def __init__(self):
        super().__init__('people_tracker')
        self.subscription = self.create_subscription(LaserScan,'/scan', self.scan_callback, 10)
        self.obstacles = {}
        self.num_background_obstacles = 4
        self.next_obstacle_id = 0
        self.distance_threshold = 1.0  # Threshold to match clusters to existing people
        self.startup_count = 0

    def polar_to_cartesian(self, ranges, angle_min, angle_increment):
        coords = []
        for i, range in enumerate(ranges):
            if range < 10.0:  # Filter out distant points
                angle = angle_min + i * angle_increment
                x = range * math.cos(angle)
                y = range * math.sin(angle)
                coords.append([x, y])
        return np.array(coords)

    def scan_callback(self, msg):
        self.startup_count += 1
        points = self.polar_to_cartesian(msg.ranges, msg.angle_min, msg.angle_increment)
        clustering = DBSCAN(eps=0.5, min_samples=3).fit(points)
        labels = clustering.labels_

        # Process clusters and track people
        unique_labels = set(labels)
        for k in unique_labels:
            if k == -1:  # Ignore noise
                continue

            class_member_mask = (labels == k)
            cluster = points[class_member_mask]
            cluster_center = np.mean(cluster, axis=0)
            
            # Match with existing obstacle or create new one
            matched = False
            for obstacle_id, obstacle in self.obstacles.items():
                if np.linalg.norm(obstacle.position - cluster_center) < self.distance_threshold:
                    obstacle.update_position(cluster_center)
                    matched = True
                    break

            if not matched:
                new_obstacle = Obstacle(self.next_obstacle_id, cluster_center)
                self.obstacles[self.next_obstacle_id] = new_obstacle
                if self.startup_count < 15:
                    self.num_background_obstacles = len(self.obstacles)
                self.next_obstacle_id += 1
                self.get_logger().info(f"\tTotal people tracked: {len(self.obstacles) - self.num_background_obstacles}")

        

def main(args=None):
    rclpy.init(args=args)
    people_tracker = PeopleTracker()
    rclpy.spin(people_tracker)
    people_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
