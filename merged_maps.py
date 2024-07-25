import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import yaml
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class OccupancyGridPublisher(Node):
    def __init__(self):
        super().__init__('map_merging')
        # Define QoS profile
        self.qos_profile_v1 = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
                # Define QoS profile
        self.qos_profile_v2 = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        # Publisher for the merged map
        self.semantic_map_publisher = self.create_publisher(OccupancyGrid, 'map', self.qos_profile_v1)
        
        # Subscriber for partial maps
        self.partial_map_subscriber = self.create_subscription(OccupancyGrid, 'occupancy_grid_v1', self.partial_map_callback, self.qos_profile_v2)

        self.grid = None
        self.map_info = None

    def partial_map_callback(self, msg):
        self.get_logger().info("Received partial map")

        if self.grid is None:
            self.get_logger().info("Initializing grid")
            self.initialize_grid(msg.info)

        # Merge the incoming partial map into the global grid
        self.get_logger().info("Merging partial map")
        self.merge_partial_map(msg)
        
        # Publish the updated semantic map
        self.get_logger().info("Publishing semantic map")
        self.publish_semantic_map()

    def initialize_grid(self, map_info):
        self.map_info = map_info
        self.grid = np.full((map_info.height, map_info.width), -1)
        self.get_logger().info(f"Initialized grid with dimensions: {map_info.height}x{map_info.width}")

    def merge_partial_map(self, partial_map):
        partial_data = np.array(partial_map.data).reshape(partial_map.info.height, partial_map.info.width)
        for row in range(partial_map.info.height):
            for col in range(partial_map.info.width):
                if partial_data[row, col] != -1:
                    self.grid[row, col] = max(self.grid[row, col], partial_data[row, col])
        self.get_logger().info("Merged partial map into the global grid")

    def publish_semantic_map(self):
        # Create OccupancyGrid message
        occupancy_grid_msg = OccupancyGrid()
        occupancy_grid_msg.header = Header()
        occupancy_grid_msg.header.stamp = self.get_clock().now().to_msg()
        occupancy_grid_msg.header.frame_id = 'map'
        occupancy_grid_msg.info = self.map_info
        occupancy_grid_msg.data = self.grid.flatten().tolist()

        self.semantic_map_publisher.publish(occupancy_grid_msg)
        self.get_logger().info("Published updated semantic map")

def main(args=None):
    rclpy.init(args=args)
    occupancy_grid_publisher = OccupancyGridPublisher()
    rclpy.spin(occupancy_grid_publisher)

    # Clean up
    occupancy_grid_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
