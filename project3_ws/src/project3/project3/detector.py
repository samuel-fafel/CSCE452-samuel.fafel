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
        self.movement_threshold = 0.1
        self.history = []

    def update_position(self, new_position):
        self.history.append(self.position)
        self.position = new_position

    def is_moving(self):
        if len(self.history) >= 2:
            total_displacement = 0.0
            for i in range(1, len(self.history)):
                prev_pos = self.history[i - 1]
                curr_pos = self.history[i]
                dx = curr_pos[0] - prev_pos[0]
                dy = curr_pos[1] - prev_pos[1]
                total_displacement += (dx**2 + dy**2)**0.5
            average_displacement = total_displacement / (len(self.history) - 1)
            return average_displacement > self.movement_threshold


class PeopleTracker(Node):
    def __init__(self):
        super().__init__('people_tracker')
        self.subscription = self.create_subscription(LaserScan,'/scan', self.scan_callback, 10)
        
        self.obstacles = {}
        self.num_background_obstacles = 0 # count non-people obstacles
        self.next_obstacle_id = 0

        self.cluster_count = 5 # Minimum number of points needed to form a cluster
        self.cluster_radius = 0.3 # Points must be within this radius to be considered part of the same cluster
        self.distance_threshold = 0.75 # Threshold to match clusters to existing people
        
        self.startup_count = 0
        self.startup_LIMIT = 10
        # The purpose of self.startup_ is to initialize self.obstacles with the environmental objects, 
        # then after the _count reaches the _LIMIT, any new obstacles are counted as people.
        # In practice, we have _LIMIT scan_callbacks to find background objects and remove them from consideration before we start counting people

    def polar_to_cartesian(self, ranges, angle_min, angle_increment): # Converts to Cartesian Coords for the sake of DBSCAN
        coords = []
        for i, range in enumerate(ranges):
            if range < 10.0:  # Filter out distant points
                angle = angle_min + i * angle_increment
                x = range * math.cos(angle)
                y = range * math.sin(angle)
                coords.append([x, y])
        return np.array(coords)

    def cluster_processing(self, msg):
        points = self.polar_to_cartesian(msg.ranges, msg.angle_min, msg.angle_increment)
        clustering = DBSCAN(eps=self.cluster_radius, min_samples=self.cluster_count).fit(points)
        labels = clustering.labels_

        # Process clusters and track people
        unique_labels = set(labels)
        for L in unique_labels:
            if L == -1:  # Ignore noise
                continue

            # Determine Clusters of Points
            class_member_mask = (labels == L)
            cluster = points[class_member_mask]
            cluster_center = np.mean(cluster, axis=0)
            
            # Match with existing obstacle or create new one (don't count the same obstacle twice)
            matched = False
            for obstacle_id, obstacle in self.obstacles.items():
                if np.linalg.norm(obstacle.position - cluster_center) < self.distance_threshold:
                    obstacle.update_position(cluster_center)
                    """if obstacle.is_moving():
                        self.get_logger().info(f"Obstacle {obstacle_id} is moving!")"""
                    matched = True
                    break

            if not matched:
                new_obstacle = Obstacle(self.next_obstacle_id, cluster_center) # create new obstacle
                self.obstacles[self.next_obstacle_id] = new_obstacle # add it to the dictionary
                self.next_obstacle_id += 1 # increment id
                if self.startup_count < self.startup_LIMIT: # are we still starting up?
                    self.num_background_obstacles += 1 # if so, count it as background
                #else:
        if self.startup_count > self.startup_LIMIT:
            num_people = len(self.obstacles)-self.num_background_obstacles
            print(f"People ID's: [{num_people} Total]: ", end='')
            for obstacle_id in self.obstacles:
                if obstacle_id >= self.num_background_obstacles:
                    print(obstacle_id, end=' ')
            print()

    def scan_callback(self, msg):
        self.startup_count += 1
        self.cluster_processing(msg)

        

def main(args=None):
    rclpy.init(args=args)
    people_tracker = PeopleTracker()
    rclpy.spin(people_tracker)
    people_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
