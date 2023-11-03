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
        self.history = [position] # History of positions
        self.last_seen = Clock().now()
        self.person = False # obstacle is classified as a person
        self.movement_threshold = 1 # threshold to verify movement
        self.timeout_duration = 2*10**9 # must be seen again in less than this to continue existing

    def update_position(self, new_position):
        self.history.append(self.position)
        self.last_seen = Clock().now()
        self.position = new_position

    def has_moved(self):
        if len(self.history) >= 2:
            average_x = 0.0
            average_y = 0.0
            for i in range(len(self.history)):
                prev_pos = self.history[i - 1]
                curr_pos = self.history[i]
                average_x += curr_pos[0]
                average_y += curr_pos[1]
            
            average_x = average_x / (len(self.history))
            average_y = average_y / (len(self.history))
            
            dx = self.history[-1][0] - average_x
            dy = self.history[-1][1] - average_y

            average_displacement = math.sqrt(dx**2 + dy**2)
            return average_displacement > self.movement_threshold
        return False
    
    def timeout(self):
        return (Clock().now() - self.last_seen).nanoseconds > self.timeout_duration

class PeopleDetector(Node):
    def __init__(self):
        super().__init__('people_detector')
        self.subscription = self.create_subscription(LaserScan,'/scan', self.scan_callback, 10) # subscribe to /scan
        self.cloud_publisher = self.create_publisher(PointCloud, '/person_locations', 10) # publish to /person_locations

        self.obstacles = {}
        self.num_background_obstacles = 0 # count non-people obstacles
        self.next_obstacle_id = 0

        self.CLUSTER_COUNT = 5 # Minimum number of points needed to form a cluster
        self.CLUSTER_RADIUS = 0.3 # Points must be within this radius to be considered part of the same cluster
        self.DISTANCE_THRESHOLD = 0.6 # Threshold to match clusters to existing obstacles

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
        clustering = DBSCAN(eps=self.CLUSTER_RADIUS, min_samples=self.CLUSTER_COUNT).fit(points)
        labels = clustering.labels_

        person_locations = {}
        # Process clusters and track people
        clusters = set(labels)
        for L in clusters: # For each cluster of points....
            if L == -1:  # Ignore noise
                continue

            # Determine Clusters of Points
            class_member_mask = (labels == L)
            cluster = points[class_member_mask]
            cluster_center = np.mean(cluster, axis=0)
            
            # Match with existing obstacle or create new one (don't count the same obstacle twice)
            matched = False
            self.num_background_obstacles = 0
            for obstacle_id, obstacle in self.obstacles.items(): # ....try to match cluster with known obstacle....
                if np.linalg.norm(obstacle.position - cluster_center) < self.DISTANCE_THRESHOLD:
                    obstacle.update_position(cluster_center)
                    matched = True
                    break

            if not matched: # ....otherwise create new obstacle.
                new_obstacle = Obstacle(self.next_obstacle_id, cluster_center) # create new obstacle
                self.obstacles[self.next_obstacle_id] = new_obstacle # add it to the dictionary
                self.next_obstacle_id += 1 # increment id
            
            for obstacle_id, obstacle in self.obstacles.items():
                if obstacle.has_moved():
                    obstacle.person = True
                else:
                    #obstacle.person = False
                    self.num_background_obstacles += 1

        num_people = len(self.obstacles) - self.num_background_obstacles
        print(f"People ID's: [{num_people} Total]: ", end='')
        for obstacle_id, obstacle in self.obstacles.items():
            if obstacle.person:
                person_locations[obstacle_id] = (obstacle.position)
                print(obstacle_id, end=' ')
        print()
        return person_locations
    
    def remove_undetected_obstacles(self, msg): # If an obstacle timed out, remove it from the list
        new_dict = {}
        for obstacle_id, obstacle in self.obstacles.items():
            if not obstacle.timeout():
                new_dict[obstacle_id] = obstacle
            else:
                print(f"Obstacle {obstacle_id} was not detected. Removing...")
        return new_dict

    def scan_callback(self, msg):
        person_locations = self.cluster_processing(msg)
        self.obstacles = self.remove_undetected_obstacles(msg)
        
        # Prepare the PointCloud message
        cloud_msg = PointCloud()
        cloud_msg.header.stamp = self.get_clock().now().to_msg()
        cloud_msg.header.frame_id = "laser"
        
        # Fill the points in the PointCloud message
        for person_id, position in person_locations.items():
            point = Point32()
            point.x = float(position[0])
            point.y = float(position[1])
            point.z = 0.0  # Assuming the ground plane is at z=0
            cloud_msg.points.append(point)
        
        # Publish the message
        self.cloud_publisher.publish(cloud_msg)

        
def main(args=None):
    rclpy.init(args=args)
    people_detector = PeopleDetector()
    rclpy.spin(people_detector)
    people_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
