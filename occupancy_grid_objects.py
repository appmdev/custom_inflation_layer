import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import yaml
import time

class OccupancyGridPublisher(Node):
    def __init__(self):
        super().__init__('occupancy_grid_publisher')
        self.publisher = self.create_publisher(OccupancyGrid, 'occupancy_grid_v1', 10)

    def get_adjacent_cells(self, row, col, visited, grid):
        adjacent_cells = []
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]  # Right, Left, Down, Up
        stack = [(row, col)]

        while stack:
            current_row, current_col = stack.pop()
            for dx, dy in directions:
                new_row, new_col = current_row + dx, current_col + dy
                if 0 <= new_row < len(grid) and 0 <= new_col < len(grid[0]) and grid[new_row][new_col] == 100 and (new_row, new_col) not in visited:
                    adjacent_cells.append((new_row, new_col))
                    visited.add((new_row, new_col))
                    stack.append((new_row, new_col))

        return adjacent_cells

    def find_objects(self, grid):
        visited = set()
        objects = []

        for row in range(len(grid)):
            for col in range(len(grid[0])):
                if grid[row][col] == 100 and (row, col) not in visited:
                    visited.add((row, col))
                    cells = [(row, col)]
                    cells.extend(self.get_adjacent_cells(row, col, visited, grid))
                    objects.append(cells)

        return objects

    def publish_occupancy_grid(self, object_index, object_cells, grid_height, grid_width, map_info):
        # Initialize the occupancy grid data
        occupancy_data = [-1] * (grid_height * grid_width)

        # Fill in the cells corresponding to the object with occupied cells (100)
        for row, col in object_cells:
            index = col + row * grid_width
            occupancy_data[index] = 100

        # Create OccupancyGrid message
        occupancy_grid_msg = OccupancyGrid()
        occupancy_grid_msg.header = Header()
        occupancy_grid_msg.header.stamp = self.get_clock().now().to_msg()
        occupancy_grid_msg.header.frame_id = 'map'
        occupancy_grid_msg.info = map_info
        occupancy_grid_msg.data = occupancy_data

        self.publisher.publish(occupancy_grid_msg)
        self.get_logger().info(f"Published OccupancyGrid message for object {object_index}")

        # Add a delay to ensure message processing
        time.sleep(1)  # Adjust the delay as needed

    def get_grid_dimensions(self, grid):
        height = len(grid)
        width = len(grid[0]) if grid else 0
        return height, width

    def read_grid_from_file(self, filename):
        with open(filename, 'r') as file:
            grid = []
            for line in file:
                row = []
                for num in line.strip().split():
                    if num.isdigit():
                        row.append(int(num))
                if row:
                    grid.append(row)
        return grid

    def read_map_config(self, filename):
        with open(filename, 'r') as file:
            config = yaml.safe_load(file)
        return config

    def main(self):
        # Load map data and configuration
        map_filename = 'map.txt'
        config_filename = 'map.yaml'
        grid = self.read_grid_from_file(map_filename)
        map_config = self.read_map_config(config_filename)

        height, width = self.get_grid_dimensions(grid)
        self.get_logger().info(f"Grid Height: {height}")
        self.get_logger().info(f"Grid Width: {width}")

        objects = self.find_objects(grid)
        num_objects = len(objects)
        self.get_logger().info(f"Number of objects found: {num_objects}")
        for idx, obj in enumerate(objects, start=1):
            self.get_logger().info(f"Object {idx}: {obj}")

        while True:
            # Prompt to select object for publishing occupancy grid
            selected_object = input("Enter the object index (1 to {}), or 'q' to quit: ".format(num_objects))
            
            if selected_object.lower() == 'q':
                break
            
            try:
                selected_object_index = int(selected_object)
                if 1 <= selected_object_index <= num_objects:
                    # Prepare map info using the configuration
                    map_info = OccupancyGrid().info
                    
                    x_map_config = 0.0                    
                    try: x_map_config = float(map_config['origin'][0])
                    except: x_map_config = 0.0

                    y_map_config = 0.0                    
                    try: y_map_config = float(map_config['origin'][1])
                    except: y_map_config = 0.0

                    z_map_config = 0.0                    
                    try: z_map_config = float(map_config['origin'][2])
                    except: z_map_config = 0.0

                    map_info.width = width
                    map_info.height = height
                    map_info.resolution = map_config['resolution']
                    map_info.origin.position.x = x_map_config
                    map_info.origin.position.y = y_map_config
                    map_info.origin.position.z = z_map_config
                    map_info.origin.orientation.w = 1.0

                    self.publish_occupancy_grid(selected_object_index, objects[selected_object_index - 1], height, width, map_info)
                else:
                    self.get_logger().info("Invalid object index!")
            except ValueError:
                self.get_logger().info("Invalid input! Please enter a valid object index.")


def main(args=None):
    rclpy.init(args=args)
    occupancy_grid_publisher = OccupancyGridPublisher()
    occupancy_grid_publisher.main()

    # Spin the node
    rclpy.spin(occupancy_grid_publisher)

    # Clean up
    occupancy_grid_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()