from pathfinding_utils import setup, backtrack, path_cost            # Importing necessary functions from 'pathfind' module
from numpy import inf                                       # Importing 'inf' constant from NumPy library

def dijkstra(grid, start, goal):
    
    # Calling the 'setup' function from 'pathfind' module and assigning the returned values to variables
    path, steps, found, Row, Col, visited, parent, rq, cq, grid, stack = setup(grid, start, goal)
    
    # Creating a 2D list 'cost' initialized with 'inf' for all elements
    cost = [[inf for i in range(Col)] for j in range(Row)]                

    queue = []                                              # Creating an empty queue list
    queue.append([0, start[0], start[1]])                   # Adding the starting node with a cost of 0 to the queue
    distance = 0                                            # Initializing distance as 0

    while len(queue) > 0:                                   # Continue the loop until the queue is empty

        if found == True:                                   # If the goal is found, exit the loop
            break
        
        temp = queue.pop(0)                                 # Remove and retrieve the first element from the queue
        r = temp[1]                                         # Row index of the current node
        c = temp[2]                                         # Column index of the current node
        
        dr = [1, 0, -1, 0]                                  # Directions: Down, Right, Up, Left
        dc = [0, 1, 0, -1]
        
        for i in range(4):                                  # Explore each direction
            
            rr = r + dr[i]                                  # New row index based on the current direction
            cc = c + dc[i]                                  # New column index based on the current direction
            
            distance = abs(rr - start[0]) + abs(cc - start[1])      # Calculate the Manhattan distance from the start node
            
            if rr < 0 or cc < 0 or rr >= Row or cc >= Col:          # If the new indices are out of bounds, continue to the next iteration
                continue
            
            if visited[rr][cc] == True:                             # If the node has already been visited, continue to the next iteration
                continue
            
            if grid[rr][cc] == 1:                                   # If the node is an obstacle, continue to the next iteration
                continue

            parent[rr][cc] = [r, c]                                 # Update the parent node of the current node
            distance = path_cost([rr, cc], start, parent)           # Calculate the path cost from the start node to the current node
            cost[rr][cc] = distance                                 # Update the cost of reaching the current node
            queue.append([cost[rr][cc], rr, cc])                    # Add the current node to the queue with its cost and indices
            visited[rr][cc] = True                                  # Mark the current node as visited
            steps = steps + 1                                       # Increment the steps taken
            
            if grid[rr][cc] == "G":                                 # If the current node is the goal, set 'found' flag to True and exit the loop
                found = True
                break
        
        queue.sort()                                                # Sort the queue based on the cost of reaching each node
    
    path, count = backtrack(goal, parent, path)                     # Call the 'backtrack' function to retrieve the shortest path and its length
    
    if found:
        print("Using Dijkstra")
        print("Steps taken to find the path:", steps)
        print("Shortest path:", count, "\n")
    else:
        print("No path found.")

    return path                                                      # Return the shortest path
