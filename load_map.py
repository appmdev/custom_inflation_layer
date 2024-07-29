import rclpy
from rclpy.node import Node
from nav2_msgs.srv import LoadMap

class MapLoader(Node):
    def __init__(self):
        super().__init__('map_loader')
        self.cli = self.create_client(LoadMap, '/map_server/load_map')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = LoadMap.Request()

    def send_request(self, map_url):
        self.req.map_url = map_url
        self.future = self.cli.call_async(self.req)

def load_map_from_file(map_url):
    rclpy.init()
    map_loader = MapLoader()
    map_loader.send_request(map_url)

    while rclpy.ok():
        rclpy.spin_once(map_loader)
        if map_loader.future.done():
            try:
                response = map_loader.future.result()
                map_loader.get_logger().info(f"Map loaded: {response.result}")
            except Exception as e:
                map_loader.get_logger().info(f'Service call failed: {e}')
            break

    map_loader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    map_url = '/home/m/map/merged_map.yaml'
    load_map_from_file(map_url)