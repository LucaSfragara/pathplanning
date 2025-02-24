import numpy as np
import matplotlib.pyplot as plt
from matplotlib import patches
from construct_map import construct_map
import itertools
import heapq
from typing import List, Tuple
import math

from skimage import measure, draw
import cv2

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


OBSTACLE_COORDINATES_HARD = [
    # Obstacle 1
    [(44, 4), (44 + 18/math.sqrt(2), 4 + 18/math.sqrt(2))],  # BR
    [(44 + 18/math.sqrt(2), 4 + 18/math.sqrt(2)), (44 + 18/math.sqrt(2) - 6/math.sqrt(2), 4 + 18/math.sqrt(2) + 6/math.sqrt(2))],  # TR
    [(44, 4), (44 - 6/math.sqrt(2), 4 + 6/math.sqrt(2))],  # LB
    [(44 - 6/math.sqrt(2), 4 + 6/math.sqrt(2)), (44 - 6/math.sqrt(2) + 18/math.sqrt(2), 4 + 6/math.sqrt(2) + 18/math.sqrt(2))],  # LT

    # Obstacle 2
    [(23, 19), (23 + 10.25, 19)],  # Bottom
    [(23 + 10.25, 19), (23 + 10.25, 19 + 9)],  # Right
    [(23 + 10.25, 19 + 9), (23, 19 + 9)],  # Top
    [(23, 19 + 9), (23, 19)],  # Left

    # Obstacle 3
    [(6, 34.5), (6 + 6, 34.5)],  # Bottom
    [(6 + 6, 34.5), (6 + 6, 34.5 + 6)],  # Right
    [(6 + 6, 34.5 + 6), (6, 34.5 + 6)],  # Top
    [(6, 34.5 + 6), (6, 34.5)],  # Left

    # Obstacle 4
    [(40, 50), (40 + 18/math.sqrt(2), 50 - 18/math.sqrt(2))],  # TR
    [(40 + 18/math.sqrt(2), 50 - 18/math.sqrt(2)), (40 + 18/math.sqrt(2) - 6/math.sqrt(2), 50 - 18/math.sqrt(2) - 6/math.sqrt(2))],  # RB
    [(40, 50), (40 - 6/math.sqrt(2), 50 - 6/math.sqrt(2))],  # LT
    [(40 - 6/math.sqrt(2), 50 - 6/math.sqrt(2)), (40 - 6/math.sqrt(2) + 18/math.sqrt(2), 50 - 6/math.sqrt(2) - 18/math.sqrt(2))],  # LB
]


class VisibilityGraph:
    
    
    def __init__(self, map: np.array, obstacle_vertexes: list, start_point: tuple, end_point: tuple, resolution: int, ax, robot_width):
        
        self.map = map #(H, W, C), where C is False if free space and X and True if Obstacle
        self.h, self.w, _ = map.shape
        self.resolution = resolution
        self.start = tuple(x*resolution for x in start_point)
        self.end = tuple(x*resolution for x in end_point)
        self.ax = ax
        
        #print(obstacle_vertexes)
        #print("________")
        
        if robot_width is not None: 
            obstacle_vertexes_enlarged = self._find_vertixes(ax)
            self.flattened_vertexes =  np.vstack(obstacle_vertexes_enlarged, dtype = int)
            self.vertexes = obstacle_vertexes_enlarged
        else: 
            self.flattened_vertexes = np.array(list(itertools.chain(*obstacle_vertexes)), dtype = int)
            self.vertexes = obstacle_vertexes

            
        additional_points =  np.array([start_point, end_point]) * resolution
        
        self.flattened_vertexes = np.vstack([additional_points, self.flattened_vertexes])
        
        #scale vertexes: 
        self.flattened_vertexes = self.flattened_vertexes 
        
        self.n_vertexes = len(obstacle_vertexes) 

        self.graph = {} #dictionary holding nodes (as keys) and all respective connected nodes. {<SOURCE>: [(<SINK>, <DISTANCE>)]}
        
        self._fill_graph(ax)
        
    def djistra_shortest_path(self, ax = None) -> Tuple[List, float]:
        
        """
        Returns a list containing the shortest path between the START and END NODE and the corresponding total distance
        Parameters:
        - ax: (matplotlib.pyplot ax) provide for plotting shortest path
        """
        
        graph = self.graph
            
        #Priority Queue (min-heap) for selecting node with smallest distance efficiently
        pq = [(0, self.start)] #distance, node NOTE: HEAPQ orders element based on the first element of the tuple
        
        distances = {node: np.inf for node in self.graph.keys()} #initially set all the distances to infinity

        previous_node = {node: None for node in self.graph.keys()} #contain pointer to "best" previous node for each node. Used for path reconstruction
        
        while pq:
    
            current_distance, current_node = heapq.heappop(pq) #get smallest item
            if current_distance > distances[current_node]: #if we found a shorter path than the current one, skip
                continue
            
            for neighbor, weight in graph[current_node]:
                
                distance = current_distance + weight #get total distance
                
                if distance < distances[neighbor]: #if current distance smaller than distance so far, update the dict
                    distances[neighbor] = distance
                    heapq.heappush(pq, (distance, neighbor))
                    previous_node[neighbor] = current_node
                    
        #reconstruct path given a dictionary of nodes and the best previous node
        path = []
        current = self.end #pointer is set to end
        
        while current is not self.start: 
            
            path.append(current)
            current = previous_node[current]
        
        path.reverse() #invert the path to go from START to END
        path.insert(0, self.start)
        print(path)
        if ax:
            for i in range(len(path)-1):
                ax.plot([path[i][0], path[i+1][0]], [path[i][1], path[i+1][1]], "w-", linewidth = 1.5)
                pass 
        return path, distances[self.end]
        
    def _find_vertixes(self, ax = None):
        
        
        self.map_squeeze = np.squeeze(self.map, axis=-1)
        contours = measure.find_contours(self.map_squeeze, level=0.5)
        
        polygons = []
    
        for contour in contours:
            # OpenCV expects points in (x, y) order (i.e., (col, row)), so flip the coordinates.
            contour_xy = np.fliplr(contour).astype(np.float32)
            
            # Reshape to the format required by cv2.approxPolyDP: (N, 1, 2)
            contour_xy = contour_xy.reshape((-1, 1, 2))
            
            # Approximate the contour to a polygon with the specified tolerance.
            approx = cv2.approxPolyDP(contour_xy, 2, True)
            
            # Reshape to a simple list of points and flip back to (row, col) order.
            approx = approx.reshape(-1, 2)
            approx_rc = np.fliplr(approx).astype(int)
            
            # Convert the array of points to a list of tuples.
            polygon = [tuple(pt) for pt in approx_rc]
            polygon = np.array(polygon)
        
            
            for vertex in polygon: 
                    circle = patches.Circle(vertex[::-1], 0.3*self.resolution ,edgecolor = "r", linewidth = 0.3*self.resolution, facecolor = "white")
                    ax.add_patch(circle)
                    
                    if vertex[::-1][0] < 0 or vertex[::-1][0] > self.w-5 or vertex[::-1][1] < 0 or vertex[::-1][1] > self.h-5:
                        print(f"vertex {vertex[::-1]} is out of bound")
                    else: 
                        polygons.append(vertex[::-1])
            
            
        print(polygons)
        return polygons

    
    def _fill_graph(self, ax = None):
        
        #compute cartesian product to get all possible combinations of vertices
        vertexes_to_evaluate = list(itertools.product(self.flattened_vertexes, self.flattened_vertexes))

        #evaluate all pair to check if it is a valid one. If it is, set that cell to the distance between the two points (i.e. the weight)
        for pair in vertexes_to_evaluate: 
            
            if np.all(pair[0] == pair[1]): 
                continue
            
            if self._is_valid_line(pair[0], pair[1]):
                vertex1, vertex2 = pair
                
                #convert vertexes from array to tuple, which is hashable and we can use it as dict key
                vertex1 = (vertex1[0].item(),vertex1[1].item()) 
                vertex2 = (vertex2[0].item(), vertex2[1].item())
                
                distance = self._dist(vertex1, vertex2)
                
                if vertex1 in self.graph.keys():
                    
                    self.graph[vertex1].add((vertex2, distance)) #create element in the graph dcitionary
                
                else: 
                    self.graph[vertex1] = {(vertex2, distance)}
                
                if ax: 
                    
                    ax.plot([vertex1[0], vertex2[0]], [vertex1[1], vertex2[1]], 'w:', linewidth=0.3)  

        #get all points of the line through Bresenham line algorithm
                       
    def _is_valid_line(self,  p1: tuple, p2: tuple, ax = None) -> bool:
        """
        Check if the the line connecting two vertices lies only in free space, i.e. does not go through obstacles.
        It uses Bresenham's Line Algorithm to get all the integer points on a line.

        Parameters:
        - x1, y1: Start point (integer coordinates)
        - x2, y2: End point (integer coordinates)
        - ax: ax of matplotlib.pyplot figure used to plot the line being evaluated for debugging purposes

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
            
            #Used for debug: plot current line if needed
            if ax: 
                circle = patches.Circle((x1, y1), 0.3*self.resolution ,edgecolor = "w", linewidth = 0.3*self.resolution, facecolor = "none")
                ax.add_patch(circle)
            
            if np.all(self.map[y1-1:y1+2, x1-1:x1+2]): #return False if there is an obstacle
                return False
            
        return True

    def __repr__(self) -> np.ndarray:
        
        return self.map
    
    def display(self, ax) -> None:

        #Plot start and end
        circle_start = patches.Circle(self.start, 0.3*self.resolution ,edgecolor = "w", linewidth = 0.3*self.resolution, facecolor = "white")
        ax.add_patch(circle_start)
        
        circle_end = patches.Circle(self.end, 0.3*self.resolution ,edgecolor = "w", linewidth = 0.3*self.resolution, facecolor = "white")
        ax.add_patch(circle_end)
        
        ax.imshow(self.map)
       
        
    def _dist(self, p1: tuple, p2: tuple) -> np.float32: 
        """
        Calculates euclidean distance between two points
        
        Parameters: 
        - p1 (tuple): (x, y) coordinates of the first vertex
        - p2 (tuple): (x, y) coordinates of the second vertex
        
        Returns: 
        - distance (np.float32)
        """
        
        return np.sqrt((p1[0]-p2[0])**2 + (p1[1] - p2[1])**2).item()

if __name__ == "__main__":
    
    #Initialize Visibility Graph
    fig, ax = plt.subplots(1)
    
    RESOLUTION = 4
    START = (10,49) #in inches
    END = (65,10) #in inches
    ROBOT_WIDTH = 10
    
    map = construct_map(isEasy=True, resolution=RESOLUTION,enlarge=True, robot_width=ROBOT_WIDTH)
    graph = VisibilityGraph(map, OBSTACLE_COORDINATES_EASY, START, END, RESOLUTION, ax, robot_width=ROBOT_WIDTH)
    graph.djistra_shortest_path(ax)
    plt.title(f"Map (Axis are in pixels. 1 inch = {RESOLUTION} pixels)")
    graph.display(ax)
    plt.show()
    
