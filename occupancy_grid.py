import numpy as np
import cv2
import math
# Removed import from construct_map

class OccupancyGrid:
    # Keep 'enlarge' and 'robot_width' parameters
    def __init__(self, low_corner, high_corner, resolution, enlarge=False, robot_width=None):
        self.low_corner = np.array(low_corner)
        self.high_corner = np.array(high_corner)
        self.resolution = resolution
        self.grid_size = np.ceil((self.high_corner - self.low_corner) * self.resolution).astype(int)
        self.grid = np.zeros((self.grid_size[1], self.grid_size[0]), dtype=np.uint8) 

        # Define base obstacle vertices directly
        # Each obstacle is a list of (x, y) tuples
        base_obstacles_vertices = [
            [(7, 5), (9, 5), (9, 7), (7, 7)], # Obstacle 1
            [(2, 4), (4, 4), (4, 6), (2, 6)], # Obstacle 2
            [(5, 1), (6, 1), (6, 3), (5, 3)], # Obstacle 3
            [(6, 2), (7, 2), (7, 3), (6, 3)]  # Obstacle 4 
        ]

        obstacle_vertex_list = []
        if enlarge and robot_width is not None and robot_width > 0:
            margin = robot_width / 2.0
            for vertices in base_obstacles_vertices:
                enlarged_vertices = self._enlarge_obstacle_vertices(vertices, margin)
                obstacle_vertex_list.append(enlarged_vertices)
        else:
            obstacle_vertex_list = base_obstacles_vertices # Use base obstacles if not enlarging

        self._populate_obstacles(obstacle_vertex_list)

    def _enlarge_obstacle_vertices(self, vertices, margin):
        """Enlarges a convex polygon by pushing vertices out from the centroid."""
        if not vertices:
            return []
        
        # Calculate centroid
        arr = np.array(vertices)
        centroid = np.mean(arr, axis=0)
        cx, cy = centroid[0], centroid[1]

        enlarged_vertices = []
        for vx, vy in vertices:
            dx = vx - cx
            dy = vy - cy
            dist = math.sqrt(dx*dx + dy*dy)
            
            if dist > 1e-6: # Avoid division by zero if vertex is at centroid
                # Normalized vector from centroid to vertex
                nx = dx / dist
                ny = dy / dist
                # New vertex position pushed outwards by margin
                new_vx = vx + nx * margin
                new_vy = vy + ny * margin
            else: # Keep vertex if it's (almost) at the centroid
                new_vx, new_vy = vx, vy
                
            enlarged_vertices.append((new_vx, new_vy))
            
        return enlarged_vertices

    def world_to_grid(self, world_coords):
        # Convert world coordinates to grid coordinates
        if world_coords is None:
            return None
        # Calculate grid coordinates
        grid_coords_float = (np.array(world_coords) - self.low_corner) * self.resolution
        # Floor the coordinates and convert to int
        grid_coords = np.floor(grid_coords_float).astype(int)
        
        # Return as (x, y) tuple for consistency, but note numpy indexing is often (row, col) -> (y, x)
        # Ensure coordinates are within valid range before indexing later
        # Let's return (x, y) as calculated, drawing function will handle numpy indexing
        return tuple(grid_coords)

    # Modify to accept a list of vertex lists
    def _populate_obstacles(self, obstacle_vertex_list):
        # Populate the grid with obstacles
        for vertices in obstacle_vertex_list:
            # Convert world coordinate vertices to grid coordinates
            grid_vertices = [self.world_to_grid(v) for v in vertices]
            
            # Filter out None values 
            valid_grid_vertices = [gv for gv in grid_vertices if gv is not None]

            if len(valid_grid_vertices) >= 3: # Need at least 3 vertices for a polygon
                pts = np.array(valid_grid_vertices, dtype=np.int32)
                # Use cv2.fillPoly to draw the filled obstacle polygon
                cv2.fillPoly(self.grid, [pts], 1) # Draw filled polygon with value 1

    def is_occupied(self, world_coords):
        # Check if a given world coordinate is occupied
        grid_coords = self.world_to_grid(world_coords)
        
        if grid_coords is None:
             # If conversion fails, treat as occupied (outside defined area)
             return True

        gx, gy = grid_coords

        # Check if the grid coordinates are within the grid boundaries
        # Remember grid shape is (height, width) -> (grid_size[1], grid_size[0])
        # So, check 0 <= gy < grid_size[1] and 0 <= gx < grid_size[0]
        if 0 <= gy < self.grid_size[1] and 0 <= gx < self.grid_size[0]:
            # Return True if the cell value is 1 (occupied), False otherwise
            # Numpy indexing is [row, col] -> [gy, gx]
            return self.grid[gy, gx] == 1 
        else:
            # Consider coordinates outside the grid boundaries as occupied
            return True

    def get_boolean_grid(self):
        """Returns the occupancy grid as a boolean numpy array."""
        return self.grid.astype(bool)


    def get_grid_for_display(self):
        """Returns a grid suitable for display (e.g., with matplotlib)."""
        # Convert 0s (free) to 1 (white) and 1s (occupied) to 0 (black) for typical display
        display_grid = 1 - self.grid 
        return display_grid

# Example usage (optional, for testing)
if __name__ == '__main__':
    import matplotlib.pyplot as plt

    low_corner = (0, 0)
    high_corner = (13, 9) # Match MAP_WIDTH, MAP_HEIGHT from construct_map
    #resolution = 1.0 / 8.0 # Match resolution = 8 points per inch
    resolution = 8
    # Create grid without enlargement
    occ_grid = OccupancyGrid(low_corner, high_corner, resolution, enlarge=False)

    # Get the grid for display (0=occupied/black, 1=free/white)
    display_grid_data = occ_grid.get_grid_for_display()

    print(f"Grid shape: {display_grid_data.shape}") # Should be (height, width) -> (72, 104) for resolution 8

    plt.figure()
    # Use origin='lower' to match world coordinates (y increases upwards)
    # extent defines the world coordinates corresponding to the grid boundaries
    plt.axis([low_corner[0], high_corner[0], low_corner[1], high_corner[1]])
    plt.imshow(display_grid_data, cmap='gray', origin='lower', 
               extent=[low_corner[0], high_corner[0], low_corner[1], high_corner[1]])
    plt.title("Occupancy Grid (OpenCV Filled)")
    plt.xlabel("X world coordinate")
    plt.ylabel("Y world coordinate")
    plt.axis('equal') # Ensure aspect ratio is correct
    plt.show()

    # Test with enlargement
    occ_grid_enlarged = OccupancyGrid(low_corner, high_corner, resolution, enlarge=True, robot_width=0.5) # Example robot width
    display_grid_enlarged = occ_grid_enlarged.get_grid_for_display()
    
    plt.figure()
    plt.imshow(display_grid_enlarged, cmap='gray', origin='lower',
               extent=[low_corner[0], high_corner[0], low_corner[1], high_corner[1]])
    plt.title("Occupancy Grid (Enlarged, OpenCV Filled)")
    plt.xlabel("X world coordinate")
    plt.ylabel("Y world coordinate")
    plt.axis('equal')
    plt.show()
