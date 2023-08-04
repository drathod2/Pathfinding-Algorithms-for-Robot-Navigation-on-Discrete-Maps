# Import Libraries
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import csv
import numpy as np
from numpy import inf 
import math


def load_map(file_path):                           
    with open(file_path, 'r') as map_file:
        reader = csv.reader(map_file)                        # Create a CSV reader object to read the file
        rows = list(reader)                                  # Convert the reader object to a list of rows

    start = [int(rows[0][1]), int(rows[0][2])]               # Extract the start point coordinates
    goal = [int(rows[1][1]), int(rows[1][2])]                # Extract the goal point coordinates
    grid = [[int(col) for col in row] for row in rows[2:]]   # Convert the remaining rows to a grid

    return grid, start, goal                                 # Return the grid, start point, and goal point

def visualize_path(grid, path, title, start, goal):
    # Visualization of the found path using matplotlib
    fig, ax = plt.subplots(1)
    ax.margins()

    # Draw map
    row = len(grid)     # Get the number of rows in the grid (map size)
    col = len(grid[0])  # Get the number of columns in the grid (map size)
    for i in range(row):
        for j in range(col):
            if grid[i][j]:
                ax.add_patch(Rectangle((j, i), 1, 1, edgecolor='k', facecolor='k', linewidth=1.0))  # Draw obstacle with thicker edges
            else:
                ax.add_patch(Rectangle((j, i), 1, 1, edgecolor='k', facecolor='w', linewidth=1.0))  # Draw free space with regular edges

    # Draw path
    for x, y in path:
        ax.add_patch(Rectangle((y, x), 1, 1, edgecolor='k', facecolor='g', linewidth=1.0))  # Draw path with thicker edges
    ax.add_patch(Rectangle((start[1], start[0]), 1, 1, edgecolor='k', facecolor='b', linewidth=1.0))  # Draw start point with thicker edges
    ax.add_patch(Rectangle((goal[1], goal[0]), 1, 1, edgecolor='k', facecolor='b', linewidth=1.0))  # Draw goal point with thicker edges

    # Add text labels for start and goal
    ax.text(start[1] + 0.5, start[0] + 0.5, "Start", ha='center', va='center', color='w', weight='bold', fontsize=7)
    ax.text(goal[1] + 0.5, goal[0] + 0.5, "Goal", ha='center', va='center', color='w', weight='bold', fontsize=7)

    # Graph settings
    plt.title(title)  # Set the title of the plot
    plt.axis('scaled')  # Set the aspect ratio of the plot
    plt.xlim(0, col)  # Set the x-axis limits
    plt.ylim(row, 0)  # Set the y-axis limits (inverted to start from the top)

def setup(grid, start, goal):
    epsi = 3
    distance = 0
    path = []                                         # Initialize an empty path
    stack = []                                        # Initialize an empty stack
    steps = 0                                         # Initialize step count to 0
    found = False                                     # Initialize found flag to False
    rq = []                                           # Initialize row queue
    cq = []                                           # Initialize column queue
    stack.append(start)
    rq.append(start[0])                                   # Add the start row to the row queue
    cq.append(start[1])                                   # Add the start column to the column queue

    Row = len(grid)                                       # Get the number of rows in the grid
    Col = len(grid[0])                                    # Get the number of columns in the grid

    if start[0] == goal[0] and start[1] == goal[1]:         # Check if the start and goal are at the same point
        found = True                                        
        steps = -1                                          # Set steps to -1 to indicate they are the same point

    grid[goal[0]][goal[1]] = "G"                           # Assign "G" as the goal name on the grid

    visited = [[False for j in range(Col)] for i in range(Row)]      # Create a boolean array of size RxC to check if a node has been visited
    visited[start[0]][start[1]] = True                               # Mark the start node as visited
    steps = steps + 1                                                # Increment steps by 1 since the start node is visited

    parent = [[0 for i in range(Col)] for j in range(Row)]              # Create an array to store the parent node of the current visited node
    
    queue=[]
    queue.append([0,start[0],start[1]])

    return path, steps, found, Row, Col, visited, parent, rq, cq, grid, stack, epsi, distance, queue  

def backtrack(goal, parent, path):
    count = 0
    position = [goal[0], goal[1]]                 # Start backtracking from the goal position
    while position != 0:                          # Continue backtracking until reaching the starting position
        count += 1                                 # Increment the count of steps taken during backtracking
        path.append(position)                      # Append the current position to the path
        position = parent[position[0]][position[1]]           # Update the current position using the parent information
    path = path[::-1]                                         # Reverse the path to obtain the correct order

    return path, count

def path_cost(point, start, parent):
    cost = 0                                            # Initialize cost variable

    current_pos = point                                 # Store the current point

    
    while current_pos != start:                                   # Traverse the parent nodes from current point until start point is reached
        cost = cost + 1                                           # Increment the cost by 1
        current_pos = parent[current_pos[0]][current_pos[1]]      # Update the current point to its parent
    
    return cost                                                   # Return the calculated cost