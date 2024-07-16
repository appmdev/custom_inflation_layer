#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
import numpy as np

class Mapper(Node):
    def __init__(self):
        super().__init__('pcl2_to_costmap')

        # Initialize map response
        self.response = None

        # Create a service client to request map data
        self.map_client = self.create_client(GetMap, '/map_server/map')
        while not self.map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Map server service not available, waiting...')
        
        # Request map data
        self.request_map_data()

    def request_map_data(self):
        request = GetMap.Request()
        future = self.map_client.call_async(request)
        future.add_done_callback(self.map_data_callback)

    def map_data_callback(self, future):
        try:
            self.response = future.result()
            self.get_logger().info('Map data received from the map server!')
            self.process_map_data()
        except Exception as e:
            self.get_logger().info('Failed to receive map data: %r' % (e,))
        
    def process_map_data(self):
        if self.response is None:
            self.get_logger().info('No map data to process.')
            return
        
        # Extract occupancy grid data
        grid_map = np.array(self.response.map.data).reshape((self.response.map.info.height, self.response.map.info.width))

        # Save map data to a text file
        self.save_map_data(grid_map, 'map_data.txt')



    def save_map_data(self, grid_map, filename):
        with open(filename, 'w') as f:
            for row in grid_map:
                f.write(' '.join(map(str, row)) + '\n')
        self.get_logger().info(f"Map data saved to {filename}")


def main():
    rclpy.init()
    node = Mapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
