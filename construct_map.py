"""
File:           construct_map.py
Date:           2/12/2025
Description:    lab5 code for contructing a map from obstacle coordinates 
Author:         Joshua Samulak (jsamulak@andrew.cmu.edu)

"""

import math
import numpy
import matplotlib.pyplot as plt


"""
#! IT IS HIGHLY RECOMMENDED YOU EDIT THIS FILE TO IMPLEMENT A CONFIGURATION SPACE
#! HOW YOU DO THAT IS FULLY UP TO YOU! :D

example code is at the bottom of the file
"""

class Line:
    # line defined by p1, p2; each point = (X, Y)
    # boundType:
    # ** information of what type of boundary is it? See valid inputs below
    # "T", "B", "R", "L" - only a Top, Bottom, Right, or Left line respectively
    # "TL" or "LT" - Top and Left line
    # "TR" or "RT" - Top and Right line
    # "BL" or "LB" - Bottom and Left line
    # "BR" or "RB" - Bottom and Right line
    def __init__(self, p1, p2, boundType):
        self.p1 = p1
        self.p2 = p2
        self.boundType = boundType
        pass
    
    # This function, given an (x, y) coordinate,
    # the line checks to see if, relative to itself, the said coordinate is
    # Above itself (in the case of it having boundType "B", ie bottom line)
    # Below itself (in the case of it having boundType "T", ie Top line)
    # Right of itself (in the case of it having boundType "L", ie Left line)
    # or Left of itself (in the case of it having boundType "R", ie Right line)
    def inLine(self, x, y):
        inLine = True
        y_on_line = 0 # gets changed
        x_on_line = 0 # gets changed

        # in the case of a vertical line, correct code shouldn't input "T" or "B"
        # so we don't care about "y_on_line"
        if (self.p1[0] == self.p2[0]): # vertical line have same x
            y_on_line = 0
            x_on_line = self.p1[0]
        # in the case of a horzontal line, correct code shouldn't input "R" or "L"
        # so we don't care about "x_on_line"
        elif (self.p1[1] == self.p2[1]):
            y_on_line = self.p1[1]
            x_on_line = 0
        # if not horizontal or vertical, then we do math to get relative xs and ys
        else:
            y_on_line = self.p1[1] + (self.p2[1] - self.p1[1]) * (x - self.p1[0]) / (self.p2[0] - self.p1[0])
            x_on_line = self.p1[0] + (self.p2[0] - self.p1[0]) * (y - self.p1[1]) / (self.p2[1] - self.p1[1])

        # using x_on_line and y_on_line, we can now easily check if the point is "within" the line
        for char in self.boundType:
            if char == 'T':
                inLine = inLine and (y < y_on_line)
            if char == 'B':
                inLine = inLine and (y > y_on_line)
            if char == 'R':
                inLine = inLine and (x < x_on_line)
            if char == 'L':
                inLine = inLine and (x > x_on_line)
            # print(char)
            # print(inLine)
        
        return inLine

class Obstacle:
    # all objects are of object type "Line"
    def __init__(self, l1, l2, l3, l4):
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.l4 = l4
        pass
    
    # returns true if x,y is in the obstacle, false otherwise
    def clash(self, x, y):

        inBounds = True
        inBounds = inBounds and self.l1.inLine(x, y)
        inBounds = inBounds and self.l2.inLine(x, y)
        inBounds = inBounds and self.l3.inLine(x, y)
        inBounds = inBounds and self.l4.inLine(x, y)

        return inBounds


def enlarge_obstacle(obstacle, robot_width):
    """
    Enlarges an obstacle by shifting its boundary lines outward.
    
    :param obstacle: Obstacle object containing four lines.
    :param robot_width: Width of the robot.
    :return: New enlarged obstacle as a list of Line objects.
    """
    inflation_margin = -robot_width / 2  # Half of robot width

    def offset_line(line):
        """Moves a line outward by the inflation margin."""
        (x1, y1) = line.p1
        (x2, y2) = line.p2

        # Compute direction vector
        dx = x2 - x1
        dy = y2 - y1
        length = math.sqrt(dx ** 2 + dy ** 2)

        if length == 0:
            return line
        # Compute unit perpendicular vector
        perp_x = -dy / length
        perp_y = dx / length

        # Move both points outward
        x1_new = x1 + inflation_margin * perp_x
        y1_new = y1 + inflation_margin * perp_y
        x2_new = x2 + inflation_margin * perp_x
        y2_new = y2 + inflation_margin * perp_y

        return Line((x1_new, y1_new), (x2_new, y2_new), line.boundType)

    # Create a new enlarged obstacle with adjusted lines
    enlarged_lines = [offset_line(obstacle.l1), offset_line(obstacle.l2), 
                      offset_line(obstacle.l3), offset_line(obstacle.l4)]
    
    return Obstacle(*enlarged_lines)


def construct_obstacles(enlarge = False, robot_width = None):
    # construct obstacles
    # Remeber that points are (X, Y)

    obstacles = [
        Obstacle(
            Line((7, 5), (9, 5), "B"),
            Line((9, 5), (9, 7), "R"),
            Line((9, 7), (7, 7), "T"),
            Line((7, 7), (7, 5), "L")
        ),
    
        Obstacle(
            Line((2, 4), (4, 4), "B"),
            Line((4, 4), (4, 6), "R"),
            Line((5, 6), (2, 6), "T"),
            Line((2, 6), (2, 4), "L")
        ),
        
        Obstacle(
            Line((5, 1), (6, 1), "B"),
            Line((5, 1), (5, 3), "L"),
            Line((5, 3), (6, 3), "T"),
            Line((6, 1 ), (6, 3), "R")
        ),
        Obstacle(
            Line((6, 2), (7, 2), "B"),
            Line((6, 2), (6, 3), "L"),
            Line((7, 3), (7, 3), "T"),
            Line((7, 2), (7, 3), "R")
        ),
        
    ]

    if enlarge: 
        obstacles_enlarged = []
        for obstacle in obstacles:
            obstacle_enlarged = enlarge_obstacle(obstacle=obstacle, robot_width = robot_width)  
            
            obstacles_enlarged.append(obstacle_enlarged)
            
        obstacles = obstacles_enlarged
        
    return obstacles #array of obstacles

# given a list of obstacles and a coordinate point,
# check if it is within the bounds of any obstacle
def check_obstacles(obstacles, x, y):
    for ob in obstacles:
        if ob.clash(x, y):
            return True
    return False


def construct_map(resolution, enlarge: bool, robot_width = None):

    obstacles = construct_obstacles(enlarge=enlarge, robot_width=robot_width)

    # Discritize points and run through them:

    # @TODO Vary RESOLUTION as desired if
    # The larger it is, the more accurate your map will be,
    # but the longer it will take

    # RESOLUTION is the number of points you want per inch
    # (anything larger than 20 takes a while)
    RESOLUTION = resolution
    # WIDTH and HEIGHT in inches of the course
    MAP_WIDTH = 13
    MAP_HEIGHT = 9

    x_disc = MAP_WIDTH * RESOLUTION
    y_disc = MAP_HEIGHT * RESOLUTION
    
    # Discritized Image
    # numpy does y first, then x. #grey scale image
    img = numpy.zeros((y_disc, x_disc,1), dtype=bool)

    for col in range(x_disc):
        for row in range(y_disc):
            hit = check_obstacles(obstacles, col/RESOLUTION, row/RESOLUTION)
            img[row, col] = hit
    print(img)
    return img


# NOTE: Values in img are indexed (Y, X, color)
# this was to work with the plt functions,
# that for some reason start with y values, then x values (blame plt, not me)

# Example plt code using img result from construct_map:

# Don't have less than 1 resolution...
# RESOLUTION is the number of points you want per inch
# (anything larger than 20 takes a while)

if __name__ == "__main__":
    resolution = 8
    img = construct_map(resolution, enlarge = False, robot_width=5)
    
    #set x and y axis lenghts
    plt.axis([0, 13*resolution, 0, 9*resolution])
    
    plt.imshow(img, cmap=plt.cm.gray, origin='lower')
    plt.show()

