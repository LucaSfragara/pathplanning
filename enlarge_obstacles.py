import math

def enlarge_obstacles(obstacles, robot_width):
    """
    Enlarges the given obstacles by half of the robot's width.

    :param obstacles: List of obstacles in the format [[(x1, y1), (x2, y2)], ...]
    :param robot_width: The width of the robot.
    :return: List of enlarged obstacle coordinates.
    """
    inflation_margin = robot_width / 2  # Inflate by half the robot's width
    enlarged_obstacles = []

    for line in obstacles:
        (x1, y1), (x2, y2) = line

        # Compute direction vector
        dx = x2 - x1
        dy = y2 - y1
        length = math.sqrt(dx ** 2 + dy ** 2)

        if length == 0:
            continue  # Avoid division by zero (should not happen in valid input)

        # Compute unit perpendicular vector
        perp_x = -dy / length
        perp_y = dx / length

        # Move both points outward by inflation_margin
        x1_new = x1 + inflation_margin * perp_x
        y1_new = y1 + inflation_margin * perp_y
        x2_new = x2 + inflation_margin * perp_x
        y2_new = y2 + inflation_margin * perp_y

        enlarged_obstacles.append([(x1_new, y1_new), (x2_new, y2_new)])

    return enlarged_obstacles


# Example usage with robot width = 2
OBSTACLE_COORDINATES_EASY = [
    [(23, 19), (33.25, 19)],  # Bottom
    [(33.25, 19), (33.25, 28)],  # Right
    [(33.25, 28), (23, 28)],  # Top
    [(23, 28), (23, 19)],  # Left
]





robot_width = 2
enlarged_obstacles = enlarge_obstacles(OBSTACLE_COORDINATES_EASY, robot_width)
for line in enlarged_obstacles:
    print(line)
