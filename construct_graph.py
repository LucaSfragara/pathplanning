import numpy as np
import matplotlib.pyplot as plt
from matplotlib import patches
from construct_map import construct_map
import itertools

#List of (x, y) coordinates
OBSTACLE_COORDINATES_EASY = [
    # Obstacle 1
    [(8, 12), (14, 12)],  # Bottom
    [(14, 12), (14, 18)],  # Right
    [(14, 18), (8, 18)],  # Top
    [(8, 18), (8, 12)],  # Left

    # Obstacle 2
    [(18, 12), (24, 12)],  # Bottom
    [(24, 12), (24, 30)],  # Right
    [(24, 30), (18, 30)],  # Top
    [(18, 30), (18, 12)],  # Left

    # Obstacle 3
    [(16, 44), (22, 44)],  # Bottom
    [(22, 44), (22, 50)],  # Right
    [(22, 50), (16, 50)],  # Top
    [(16, 50), (16, 44)],  # Left

    # Obstacle 4
    [(38, 36), (56, 36)],  # Bottom
    [(56, 36), (56, 42)],  # Right
    [(56, 42), (38, 42)],  # Top
    [(38, 42), (38, 36)],  # Left
]


class VisibilityGraph:
    
    
    def __init__(self, map: np.array, obstacle_vertexes: list, start_point: tuple, end_point: tuple, resolution: int):
        
        self.map = map #(H, W, C), where C is False if free space and X and True if Obstacle
        self.h, self.w, _ = map.shape
        self.resolution = RESOLUTION
        
        self.flattened_vertexes = np.array(list(itertools.chain(*obstacle_vertexes)))
        additional_points =  np.array([(0, self.h), (self.w, 0), (0, self.w), (self.h, self.w), start_point, end_point])
        
        self.flattened_vertexes = np.vstack([self.flattened_vertexes, additional_points])
        
        #scale vertexes: 
        self.flattened_vertexes = self.flattened_vertexes * resolution
        
        self.vertexes = obstacle_vertexes
        self.n_vertexes = len(obstacle_vertexes) 
        self.graph = np.full((len(self.flattened_vertexes), len(self.flattened_vertexes)),np.inf) #adjacency matrix all set to infinity (no connection between vertices)
        
  
        
    def _fill_graph(self):
        
        #fills diagonal of adjacency matrix with 0s. Each element is connected to itself
        np.fill_diagonal(self.graph, 0)
        
        #compute cartesian product to get all possible combinations of vertices
        vertexes_to_evaluate = list(itertools.product(self.flattened_vertexes, self.flattened_vertexes))
        
        #check if edge does not pass through object
        
        #get all points of the line through Bresenham line algorithm
                       
    def _is_valid_line(self, p1, p2, ax):
        """
        Check if the the line connecting two vertices lies only in free space, i.e. does not go through obstacles.
        It uses Bresenham's Line Algorithm to get all the integer points on a line.

        Parameters:
        - x1, y1: Start point (integer coordinates)
        - x2, y2: End point (integer coordinates)

        Returns:
        - List of (x, y) tuples representing points on the line.
        """
        points = []  # Stores points on the line
        x1, y1 = p1
        x2, y2 = p2
        
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        
        sx = 1 if x2 > x1 else -1  # Step direction for x
        sy = 1 if y2 > y1 else -1  # Step direction for y

        err = dx - dy  # Initial error term

        while True:
            
            points.append((x1, y1))  # Add current point

            if x1 == x2 and y1 == y2:  # Stop when end point is reached
                break

            e2 = 2 * err  # Double the error term

            if e2 > -dy:  # Move in x-direction
                err -= dy
                x1 += sx

            if e2 < dx:  # Move in y-direction
                err += dx
                y1 += sy
            
            circle = patches.Circle((x1, y1), 0.3*self.resolution ,edgecolor = "w", linewidth = 0.3*self.resolution, facecolor = "none")
            ax.add_patch(circle)
            
            #if np.any(self.map[y1, x1]>0): #return False if there is an obstacle
            #    return False, (x1, y1)
            
        return True, (x1, y1)

    def __repr__(self):
        
        return self.map
    
    def display(self, ax):
        

        
        for vertex in self.flattened_vertexes:
     
            circle = patches.Circle(vertex, 0.3*self.resolution ,edgecolor = "w", linewidth = 0.3*self.resolution, facecolor = "white")
            ax.add_patch(circle)
            
        ax.imshow(self.map)
       
        
    def _dist(self, p1, p2): 
        """
        Calculates euclidean distance between two points
        
        Parameters: 
        - p1 (tuple): (x, y) coordinates of the first vertex
        - p2 (tuple): (x, y) coordinates of the second vertex
        
        Returns: 
        - distance (np.float32)
        """
        
        return np.sqrt((p1[0]-p2[0])**2 + (p1[1] - p2[1])**2)

if __name__ == "__main__":
    
    #Initialize Visibility Graph
    fig, ax = plt.subplots(1)
    
    RESOLUTION = 4
    START = (10,10) #in inches
    END = (60,4) #in inches
    
    map = construct_map(isEasy=True, resolution=RESOLUTION)
    graph = VisibilityGraph(map, OBSTACLE_COORDINATES_EASY, START, END, RESOLUTION)
    print(graph._is_valid_line(START,END, ax))
    graph.display(ax)
    plt.show()