import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
import numpy as np
from nav2_msgs import *
from nav2_msgs.msg._costmap_meta_data import CostmapMetaData 
from nav2_msgs.msg._costmap import Costmap 
from geometry_msgs.msg import Pose
from scipy.ndimage import binary_dilation
import array
from std_msgs.msg import Header  # Add this import for Header
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class GlobalCostmapNode(Node):
    def __init__(self):
        super().__init__('global_costmap_node')
        # Define QoS profile
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.global_costmap_publisher = self.create_publisher(OccupancyGrid, '/global_costmap/costmap', qos_profile)
        
        self.global_costmap_publisher = self.create_publisher(OccupancyGrid, '/merged_global_costmap', 10)
        self.submap_topics = ['/occupancy_grid_v1']
        self.submap_inflation = {'/occupancy_grid_v1': 2}
        self.submap_costmaps = {} 
        self.submap_subscribers = []
        sub = self.create_subscription(OccupancyGrid, '/occupancy_grid_v1', self.submap_callback, 10)

    def submap_callback(self, msg):
        print("Publishing global costmap")
        inflated_costmap = self.apply_inflation(msg, 2)
        inflated_occupancy_grid = self.costmap_to_occupancy_grid(inflated_costmap)
        self.global_costmap_publisher.publish(inflated_occupancy_grid)

    def apply_inflation(self, occupancy_grid, inflation_radius):
        # Convert occupancy grid to Costmap2D
        metaDataCostmapObj = CostmapMetaData()
        metaDataCostmapObj.origin = occupancy_grid.info.origin
        metaDataCostmapObj.size_x = occupancy_grid.info.width
        metaDataCostmapObj.size_y = occupancy_grid.info.height
        metaDataCostmapObj.resolution = occupancy_grid.info.resolution

        costmap = Costmap()
        costmap.metadata = metaDataCostmapObj
        occupancy_grid_data_B = array.array('B', [min(max(val, 0), 255) for val in occupancy_grid.data])
        costmap.data = occupancy_grid_data_B

        # Apply inflation
        inflated_costmap_data = self.inflate_costmap(costmap, inflation_radius)

        # Create a new Costmap2D for the inflated costmap
        inflated_costmap = Costmap()
        inflated_costmap.metadata = metaDataCostmapObj
        inflated_costmap.data = inflated_costmap_data

        return inflated_costmap
    
    def inflate_costmap(self, original_costmap, inflation_radius):
        original_array = np.array(original_costmap.data, dtype=np.uint8)
        width = original_costmap.metadata.size_x
        height = original_costmap.metadata.size_y
        original_array = original_array.reshape((height, width))

        diameter = 2 * inflation_radius + 1
        structuring_element = np.zeros((diameter, diameter), dtype=bool)
        center = (inflation_radius, inflation_radius)
        y, x = np.ogrid[:diameter, :diameter]
        mask = (x - center[0]) ** 2 + (y - center[1]) ** 2 <= inflation_radius ** 2
        structuring_element[mask] = True

        inflated_array = binary_dilation(original_array, structure=structuring_element)
        inflated_array = inflated_array.astype(np.uint8) * 99 #99 teal;	100 pink-purple;	50 purple light;	17 blue; 101 and higher green
        inflated_costmap_data = inflated_array.flatten().tolist()

        return inflated_costmap_data

    def costmap_to_occupancy_grid(self, costmap):
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header = Header()
        occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        occupancy_grid.header.frame_id = "map"  # Set your desired frame ID here
        occupancy_grid.info = MapMetaData()
        occupancy_grid.info.map_load_time = self.get_clock().now().to_msg()
        occupancy_grid.info.resolution = costmap.metadata.resolution
        occupancy_grid.info.width = costmap.metadata.size_x
        occupancy_grid.info.height = costmap.metadata.size_y
        occupancy_grid.info.origin = costmap.metadata.origin

        # Convert data to array of signed bytes
        occupancy_grid_data_B = array.array('b', [max(min(val, 127), -128) for val in costmap.data])
        occupancy_grid.data = occupancy_grid_data_B

        return occupancy_grid



def main(args=None):
    rclpy.init(args=args)
    global_costmap_node = GlobalCostmapNode()
    rclpy.spin(global_costmap_node)
    global_costmap_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
