import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from example_interfaces.msg import Int64
import numpy as np
from rclpy.clock import Clock
import math

class Person:
    def __init__(self, person_id, position):
        self.id = person_id
        self.position = position  # Position is a tuple (x, y)
        self.history = [position]  # Track the history of positions
        self.person = False
        self.last_seen = Clock().now()

    def update_position(self, new_position):
        self.history.append(new_position)
        self.last_seen = Clock().now()
        self.position = new_position

    def timeout(self, duration):
        return (Clock().now() - self.last_seen).nanoseconds > duration

    def has_moved(self, threshold):
        if len(self.history) < 5:
            return False # Not enough data
       
        average_start_x = 0
        average_start_y = 0
        for i in range(5):
            average_start_x += (self.history[i][0] / 5)
            average_start_y += (self.history[i][1] / 5)

        average_x = 0.0
        average_y = 0.0
        for i in range(len(self.history)):
            average_x += (self.history[i][0] / len(self.history))
            average_y += (self.history[i][1] / len(self.history))

        average_displacement = math.sqrt((average_x - average_start_x)**2 + (average_y - average_start_y)**2)
        
        return average_displacement > threshold


    def predict_position(self, time_delta):
        if len(self.history) < 2:
            return self.position  # Not enough data to predict

        # Calculate velocity based on the last two positions
        delta_x = self.history[-1][0] - self.history[-2][0]
        delta_y = self.history[-1][1] - self.history[-2][1]

        # Simple linear prediction
        predicted_x = self.history[-1][0] + (delta_x * time_delta.nanoseconds / 10**9) 
        predicted_y = self.history[-1][1] + (delta_y * time_delta.nanoseconds / 10**9)

        return predicted_x, predicted_y

class PeopleCounter(Node):
    def __init__(self):
        super().__init__('people_counter')
        self.location_subscription = self.create_subscription(PointCloud, '/object_locations', self.object_locations_callback,10)
        self.cloud_publisher = self.create_publisher(PointCloud, '/person_locations', 10) # publish to /person_locations
        self.count_publisher = self.create_publisher(Int64, '/person_count', 10) # publish to /person_count
        self.objects = {}
        self.next_object_id = 0
        self.num_background_obstacles = 0
        self.DISTANCE_THRESHOLD = 0.2 # Threshold to consider points as the same person
        self.MOVEMENT_THRESHOLD = 0.75 # threshold to verify movement
        self.TIMEOUT_DURATION = 10**8 # (nanoseconds) see Person.timeout()
        self.ERROR_TOLERANCE = 0.1 # Tolerance for matching objects to predicted paths

    def match_object(self, msg):
        #print("---- Matching Objects ----")
        # Extract points from the PointCloud message
        current_points = np.array([[point.x, point.y] for point in msg.points])

        # Match points with existing Person objects or create new ones
        for point in current_points:
            matched = False
            for object_id, object_ in self.objects.items(): # CASE 1: object is moving less than DISTANCE_THRESHOLD units per callback
                if np.linalg.norm(np.array(object_.position) - point) < self.DISTANCE_THRESHOLD:
                    matched = True
                    object_.update_position(point)
                    if object_.has_moved(self.MOVEMENT_THRESHOLD): 
                        object_.person = True
                        print(f"{object_id}(Case 1)")
                    break
            
            if not matched:
                for object_id, object_ in self.objects.items(): # CASE 2: object is blinking, try to predict path and match
                    if object_.person:
                        predicted_position = object_.predict_position(Clock().now() - object_.last_seen)
                        x_error = predicted_position[0] - point[0]
                        y_error = predicted_position[1] - point[1]
                        total_error = math.sqrt(x_error**2 + y_error**2)
                        if total_error < self.ERROR_TOLERANCE and not object_.timeout(self.TIMEOUT_DURATION):
                            matched = True
                            print(f"{object_id}(CASE 2)")
                            object_.update_position(point)
                            break
            
            if not matched: # CASE 3: New Object
                self.next_object_id += 1 # Simple ID assignment
                self.objects[self.next_object_id] = (Person(self.next_object_id, point))

    def object_locations_callback(self, msg):
        self.match_object(msg)

        # Determine People vs Background Objects
        people = []
        people_count = 0
        for object_id, object_ in self.objects.items():
            if object_.person:
                people_count += 1
                if not object_.timeout(self.TIMEOUT_DURATION):
                    people.append(object_)

        # Prepare Int64 message
        count_msg = Int64()
        count_msg.data = people_count

        # Prepare PointCloud message
        cloud_msg = PointCloud()
        cloud_msg.header.stamp = self.get_clock().now().to_msg()
        cloud_msg.header.frame_id = "laser"
        for person in people: # Fill the points in the PointCloud message
            point = Point32()
            point.x = float(person.position[0])
            point.y = float(person.position[1])
            point.z = 0.0
            cloud_msg.points.append(point)

        self.count_publisher.publish(count_msg) # Publish message to /person_count
        self.cloud_publisher.publish(cloud_msg) # Publish message to /person_locations

        # Log the count of people
        #self.get_logger().info(f'Unique people count: {people_count}')

def main(args=None):
    rclpy.init(args=args)
    people_counter = PeopleCounter()
    rclpy.spin(people_counter)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    people_counter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
