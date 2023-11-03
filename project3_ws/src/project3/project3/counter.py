import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud
from example_interfaces.msg import Int64
import numpy as np

class Person:
    def __init__(self, person_id, position):
        self.id = person_id
        self.position = position  # Position is a tuple (x, y)
        self.history = [position]  # Track the history of positions

    def update_position(self, new_position):
        self.history.append(new_position)
        self.position = new_position

class PeopleCounter(Node):
    def __init__(self):
        super().__init__('people_counter')
        self.location_subscription = self.create_subscription(PointCloud, '/person_locations', self.person_locations_callback,10)
        self.count_publisher = self.create_publisher(Int64, '/person_count', 10) # publish to /person_locations
        self.people = {}
        self.DISTANCE_THRESHOLD = 0.75  # Threshold to consider points as the same person

    def person_locations_callback(self, msg):
        # Extract points from the PointCloud message
        current_points = np.array([[point.x, point.y] for point in msg.points])

        # Match points with existing Person objects or create new ones
        for point in current_points:
            matched = False
            for person_id, person in self.people.items():
                if np.linalg.norm(np.array(person.position) - point) < self.DISTANCE_THRESHOLD:
                    person.update_position(point)
                    matched = True
                    break
            
            if not matched:
                new_id = len(self.people) + 1  # Simple ID assignment
                self.people[new_id] = (Person(new_id, point))

        count_msg = Int64()
        count_msg.data = len(self.people)
        self.count_publisher.publish(count_msg)

        # Log the count of people
        self.get_logger().info(f'Unique people count: {len(self.people)}')

    def is_point_unique(self, new_point):
        for person in self.people:
            if np.linalg.norm(np.array(person.position) - new_point) < self.DISTANCE_THRESHOLD:
                return False
        return True

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
