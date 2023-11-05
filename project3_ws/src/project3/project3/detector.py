import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sklearn.cluster import DBSCAN
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import numpy as np
import math
from rclpy.clock import Clock

class Obstacle:
    def __init__(self, obstacle_id, position):
        self.obstacle_id = obstacle_id
        self.position = position  # Position is a 2D tuple (x, y)

class PeopleDetector(Node):
    def __init__(self):
        super().__init__('people_detector')
        self.subscription = self.create_subscription(LaserScan,'/scan', self.scan_callback, 10) # subscribe to /scan
        self.points_publisher = self.create_publisher(PointCloud, '/object_locations', 10) # publish to /object_locations

        self.CLUSTER_COUNT = 5 # Minimum number of points needed to form a cluster
        self.CLUSTER_RADIUS = 0.3 # Points must be within this radius to be considered part of the same cluster

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
        next_obstacle_id = 0
        obstacles = {}

        # Process clusters and track people
        points = self.polar_to_cartesian(msg.ranges, msg.angle_min, msg.angle_increment)
        clustering = DBSCAN(eps=self.CLUSTER_RADIUS, min_samples=self.CLUSTER_COUNT).fit(points)
        labels = clustering.labels_
        clusters = set(labels)
        for L in clusters: # For each cluster of points....
            if L == -1:  # Ignore noise
                continue

            # Determine Clusters of Points
            class_member_mask = (labels == L)
            cluster = points[class_member_mask]
            cluster_center = np.mean(cluster, axis=0)
        
            new_obstacle = Obstacle(next_obstacle_id, cluster_center) # create new obstacle
            obstacles[next_obstacle_id] = new_obstacle # add it to the dictionary
            next_obstacle_id = len(obstacles) + 1 # increment id

        obstacle_locations = {}
        for obstacle_id, obstacle in obstacles.items():
                obstacle_locations[obstacle_id] = (obstacle.position)
        return obstacle_locations

    def scan_callback(self, msg):
        object_locations = self.cluster_processing(msg)
        
        # Prepare PointCloud message
        cloud_msg = PointCloud()
        cloud_msg.header.stamp = self.get_clock().now().to_msg()
        cloud_msg.header.frame_id = "laser"
        for obstacle_id_id, obstacle in self.obstacles.items(): # Fill the points in the PointCloud message
            point = Point32()
            point.x = float(obstacle.position[0])
            point.y = float(obstacle.position[1])
            point.z = 0.0
            cloud_msg.points.append(point)
        
        self.points_publisher.publish(cloud_msg) # Publish to /object_locations

        
def main(args=None):
    rclpy.init(args=args)
    people_detector = PeopleDetector()
    rclpy.spin(people_detector)
    people_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
