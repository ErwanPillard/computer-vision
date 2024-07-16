import networkx as nx
import numpy as np

def astar(grid, start, goal):
    # Create a graph from the grid
    G = nx.grid_2d_graph(20, 30)

    # Remove edges corresponding to obstacles in the grid
    for row in range(20):
        for col in range(30):
            if grid[row, col] == 1:
                G.remove_node((row, col))

    # Find the shortest path using A*
    path = nx.astar_path(G, start, goal)

    # Detect direction changes (right angles) in the path
    directions = []

    for i in range(1, len(path) - 1):
        prev_node = path[i - 1]
        curr_node = path[i]
        next_node = path[i + 1]

        # Calculate direction between consecutive points
        direction1 = (curr_node[0] - prev_node[0], curr_node[1] - prev_node[1])
        direction2 = (next_node[0] - curr_node[0], next_node[1] - curr_node[1])

        if direction1 != direction2:
            directions.append(curr_node)

    return path, directions
