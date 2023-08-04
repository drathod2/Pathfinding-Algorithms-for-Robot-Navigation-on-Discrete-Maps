# Import the necessary functions and modules
from pathfinding import bfs, dfs, dijkstra, astar, wastar
from pathfinding_utils import load_map, visualize_path
import matplotlib.pyplot as plt


if __name__ == "__main__":
    # Load the map
    grid, start, goal = load_map('map.csv')

    # Search
    bfs_path = bfs(grid, start, goal)
    dfs_path = dfs(grid, start, goal)
    dijkstra_path = dijkstra(grid, start, goal)
    astar_path = astar(grid, start, goal)
    wastar_path = wastar(grid, start, goal)

    # Show result
    visualize_path(grid, bfs_path, 'Breadth-First Search(BFS)', start, goal)
    visualize_path(grid, dfs_path, 'Depth-first search(DFS)', start, goal)
    visualize_path(grid, dijkstra_path, 'Dijkstra', start, goal)
    visualize_path(grid, astar_path , 'A*', start, goal)
    visualize_path(grid, wastar_path , 'WA*', start, goal)
    
    plt.show()