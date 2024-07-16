from collections import deque


def get_adjacent_cells(row, col, visited, grid):
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



def find_objects(grid):
    visited = set()
    objects = []
    
    for row in range(len(grid)):
        for col in range(len(grid[0])):
            if grid[row][col] == 100 and (row, col) not in visited:
                visited.add((row, col))
                cells = [(row, col)]
                cells.extend(get_adjacent_cells(row, col, visited, grid))
                objects.append(cells)
    
    return objects

def read_grid_from_file(filename):
    with open(filename, 'r') as file:
        grid = []
        for line in file:
            row = []
            for num in line.strip().split():
                if num.isdigit():
                    row.append(int(num))
            if row:
                grid.append(row)
    #print(grid)
    return grid



def main():
    filename = 'map_data.txt'
    grid = read_grid_from_file(filename)

    objects = find_objects(grid)
    num_objects = len(objects)
    print("Number of objects found:", num_objects)
    for idx, obj in enumerate(objects, start=1):
        print("Object", idx, ":", obj)

if __name__ == "__main__":
    main()
