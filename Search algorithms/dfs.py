from pathfinding_utils import setup, backtrack

def dfs(grid, start, goal):
    # Initialize variables and data structures
    path, steps, found, Row, Col, visited, parent, rq, cq, grid, stack = setup(grid, start, goal)

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

