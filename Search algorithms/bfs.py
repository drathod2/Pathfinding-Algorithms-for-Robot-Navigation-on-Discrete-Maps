from pathfinding_utils import setup, backtrack

def bfs(grid, start, goal):

# Initialize variables and data structures
    path, steps, found, Row, Col, visited, parent, rq, cq, grid, stack = setup(grid, start, goal)

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
