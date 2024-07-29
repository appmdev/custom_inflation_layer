import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import yaml
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from nav2_msgs.srv import LoadMap

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
        self.semantic_map_publisher = self.create_publisher(OccupancyGrid, 'map2', self.qos_profile_v1)
        
        # Subscriber for partial maps
        self.partial_map_subscriber = self.create_subscription(OccupancyGrid, 'occupancy_grid_v1', self.partial_map_callback, self.qos_profile_v2)

        # Service client for map_server
        self.map_service_client = self.create_client(LoadMap, '/map_server/load_map')
        
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
        # Save the OccupancyGrid to a temporary YAML file
        map_file_path = '/home/m/map/merged_map.yaml'
        self.save_map_to_yaml(map_file_path)
        #self.publish_semantic_map()

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
        # Save the OccupancyGrid to a temporary YAML file
        map_file_path = '/home/m/map/merged_map.yaml'
        self.save_map_to_yaml(map_file_path)
        

        # Create a LoadMap request
        request = LoadMap.Request()
        request.map_url = f'file://{map_file_path}'

        # Call the map_server load_map service
        self.get_logger().info("Waiting for map_server/load_map service...")
        self.map_service_client.wait_for_service()

        self.get_logger().info("Calling map_server/load_map service...")
        future = self.map_service_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info("Map successfully loaded by map_server")
        else:
            self.get_logger().error(f"Failed to call map_server/load_map service: {future.exception()}")

    def save_map_to_yaml(self, map_file_path):
        map_data = {
            'image': '/home/m/map/merged_map.pgm',
            'resolution': self.map_info.resolution,
            'origin': [self.map_info.origin.position.x, self.map_info.origin.position.y, self.map_info.origin.orientation.z],
            'occupied_thresh': 0.65,
            'free_thresh': 0.196,
            'negate': 0
        }

        # Save the grid data as a PGM file
        pgm_file_path = map_data['image']
        self.save_pgm_file(pgm_file_path, self.grid, self.map_info.width, self.map_info.height)

        # Save the YAML file
        with open(map_file_path, 'w') as yaml_file:
            yaml.dump(map_data, yaml_file)
        self.get_logger().info(f"Saved map to {map_file_path}")

    def save_pgm_file(self, file_path, data, width, height):
        with open(file_path, 'wb') as f:
            f.write(b'P5\n')
            f.write(f'{width} {height}\n'.encode())
            f.write(b'255\n')

            # Ensure data is within the valid byte range
            normalized_data = np.clip(data, 0, 100).astype(np.uint8)
            # Flip and mirror the data
            flipped_data = np.flipud(normalized_data)
            for value in flipped_data.flatten():
                f.write(bytes([255 - int((value / 100) * 255)]))
        self.get_logger().info(f"Saved PGM file to {file_path}")

def main(args=None):
    rclpy.init(args=args)
    occupancy_grid_publisher = OccupancyGridPublisher()
    rclpy.spin(occupancy_grid_publisher)

    # Clean up
    occupancy_grid_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
