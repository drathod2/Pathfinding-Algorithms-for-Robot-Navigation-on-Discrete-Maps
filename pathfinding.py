from pathfinding_utils import setup, backtrack, path_cost           # Importing necessary functions from 'pathfinding_utils' module
from numpy import inf

def bfs(grid, start, goal):

# Initialize variables and data structures
    path, steps, found, Row, Col, visited, parent, rq, cq, grid, stack, epsi, distance, queue = setup(grid, start, goal)

    while len(rq) > 0:                                  
        # Terminate the loop if the start and goal are the same
        if found == True:                                 
            break
        
        r = rq[0] 
        c = cq[0]
        rq.pop(0)
        cq.pop(0)
        
        dr = [1, 0, -1, 0]                            # Directions: right, down, left, up
        dc = [0, 1, 0, -1]
        
        for i in range(4):                            # Explore neighbor nodes
            rr = r + dr[i]
            cc = c + dc[i]
            
            if rr < 0 or cc < 0 or rr >= Row or cc >= Col:     # Out of grid condition
                continue
            
            if visited[rr][cc] == True:                        # If already visited
                continue
            
            if grid[rr][cc] == 1:                              # If obstacle is encountered
                continue
            
            rq.append(rr)                                      # Add new nodes to the queue for exploration (FIFO)
            cq.append(cc)
            visited[rr][cc] = True
            parent[rr][cc] = [r, c]
            steps = steps + 1
            
            if grid[rr][cc] == "G":                             # If goal is reached 
                found = True
                break
            
    path, count = backtrack(goal, parent, path)

    if found:
        print("Using BFS")
        print("Steps taken to find the path:", steps)
        print("Shortest path:", count, "\n")
    else:
        print("No path found.")

    return path

def dfs(grid, start, goal):
    # Initialize variables and data structures
    path, steps, found, Row, Col, visited, parent, rq, cq, grid, stack, epsi, distance, queue = setup(grid, start, goal)

    while len(stack) > 0:
        if found:
            break
        r, c = stack.pop()
        dr = [1, 0, -1, 0]                             # Directions: Down, Right, Up, Left
        dc = [0, 1, 0, -1]
        
        visited[r][c] = True
        steps += 1

        for i in range(4):
            rr = r + dr[i]
            cc = c + dc[i]
            if rr < 0 or cc < 0 or rr >= Row or cc >= Col:
                continue                                # Skip invalid grid positions
            if visited[rr][cc] or grid[rr][cc] == 1:
                continue                                # Skip visited positions or obstacles
            stack.append((rr, cc))                       # Add valid neighbor to stack
            visited[rr][cc] = True
            parent[rr][cc] = (r, c)                      # Update parent of the neighbor
            if grid[rr][cc] == "G":
                found = True                             # Found the goal position
                break

    path, count = backtrack(goal, parent, path)

    if found:
        print("Using DFS")
        print("Steps taken to find the path:", steps)
        print("Shortest path:", count, "\n")
    else:
        print("No path found.")

    return path

def dijkstra(grid, start, goal):
    
    # Calling the 'setup' function from 'pathfind' module and assigning the returned values to variables
    path, steps, found, Row, Col, visited, parent, rq, cq, grid, stack, epsi, distance, queue = setup(grid, start, goal)
    
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

def astar(grid, start, goal):
    # Calling the 'setup' function from 'pathfind' module and assigning the returned values to variables
    path, steps, found, Row, Col, visited, parent, rq, cq, grid, stack, epsi, distance, queue = setup(grid, start, goal)
    
    # Creating a 2D list 'cost' initialized with 'inf' for all elements
    cost = [[inf for i in range(Col)] for j in range(Row)]
    
    # Setting the heuristic cost of the start node based on the Manhattan distance to the goal
    cost[start[0]][start[1]] = (abs(start[0] - goal[0]) + abs(start[1] - goal[1]))

    while len(queue) > 0:                                                # Continue the loop until the queue is empty

        if found == True:                                                # If the goal is found, exit the loop
            break

        temp = queue.pop(0)                                              # Remove and retrieve the first element from the queue
        r = temp[1]                                                      # Row index of the current node
        c = temp[2]                                                      # Column index of the current node
        
        dr = [1, 0, -1, 0]                                               # Directions: Down, Right, Up, Left
        dc = [0, 1, 0, -1]
        steps = steps + 1  
        
        for i in range(4):                                              # Explore each direction
            
            rr = r + dr[i]                                           # New row index based on the current direction
            cc = c + dc[i]                                           # New column index based on the current direction
            
            if rr < 0 or cc < 0:                                    # If the new indices are out of bounds, continue to the next iteration
                continue
            
            if rr >= Row or cc >= Col:                             # If the new indices are out of bounds, continue to the next iteration
                continue
            
            if visited[rr][cc] == True:                              # If the node has already been visited
                # Update the cost and parent if a better path is found
                if cost[rr][cc] > path_cost([rr, cc], start, parent) + (abs(rr - goal[0]) + abs(cc - goal[1])):
                    cost[rr][cc] = path_cost([rr, cc], start, parent) + (abs(rr - goal[0]) + abs(cc - goal[1]))
                    parent[rr][cc] = [r, c]
                continue
            
            if grid[rr][cc] == 1:                                    # If the node is an obstacle, continue to the next iteration
                continue
            
            parent[rr][cc] = [r, c]                                 # Update the parent node of the current node
            distance = path_cost([rr, cc], start, parent)           # Calculate the path cost from the start node to the current node
            
            heuristic = abs(rr - goal[0]) + abs(cc - goal[1])        # Calculate the heuristic (Manhattan distance) to the goal
            cost[rr][cc] = distance + heuristic                      # Update the cost of reaching the current node (g + h)
            
            queue.append([cost[rr][cc], rr, cc])                     # Add the current node to the queue with its cost and indices
            visited[rr][cc] = True                                   # Mark the current node as visited
            
            if grid[rr][cc] == "G":                                  # If the current node is the goal, set 'found' flag to True and exit the loop
                found = True
                break
        
        queue.sort()                                                 # Sort the queue based on the cost of reaching each node

    path, count = backtrack(goal, parent, path)                      # Call the 'backtrack' function to retrieve the shortest path and its length

    if found:
        print("Using A*")
        print("Steps taken to find the path:", steps)
        print("Shortest path:", count, "\n")
    else:
        print("No path found.")

    return path                                                       # Return the shortest path

def wastar(grid, start, goal):
    path, steps, found, Row, Col, visited, parent, rq, cq, grid, stack, epsi, distance, queue = setup(grid, start, goal)
    
    cost = [[inf for i in range(Col)] for j in range(Row)]
    
    cost[start[0]][start[1]] = (abs(start[0] - goal[0]) + abs(start[1] - goal[1]))
    
    while (len(queue))>0:

        if found==True:
            break

        temp=queue.pop(0)
        r=temp[1]
        c=temp[2]
        dr=[0,+1,0,-1]
        dc=[+1,0,-1,0]
        steps=steps+1
        for i in range(4):
            rr = r + dr[i]
            cc = c + dc[i]
            if rr<0 or cc<0:
                continue
            if rr>=Row or cc>=Col:
                continue
            if visited[rr][cc]==True:
                if cost[rr][cc]>path_cost([rr,cc],start,parent) + epsi*(abs(rr-goal[0])+abs(cc-goal[1])):
                    cost[rr][cc]=path_cost([rr,cc],start,parent) + epsi*(abs(rr-goal[0])+abs(cc-goal[1]))
                    parent[rr][cc]=[r,c]
                continue
            if grid[rr][cc]==1:
                continue
            parent[rr][cc] = [r,c]
            distance=path_cost([rr,cc],start,parent)                  # cost to node g(x)
            heuristic=abs(rr-goal[0])+abs(cc-goal[1])           # h(x) as manhattan distance of node from goal 
            cost[rr][cc]=distance+epsi*heuristic                # calculate total cost using weighted heuristic
            queue.append([cost[rr][cc], rr,cc])
            visited[rr][cc]=True
            
            
            if grid[rr][cc]== "G":
                found=True
                break
        
        queue.sort()                                                    # sort queue based on total cost 

    path, count = backtrack(goal, parent, path)

    if found:
        print("Using Weighted A*")
        print("Steps taken to find the path:", steps)
        print("Shortest path:", count, "\n")
    else:
        print("No path found.")
    return path