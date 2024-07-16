import networkx as nx
import numpy as np

def astar(grid, start, goal):
    # Création d'un graphe à partir de la grille
    G = nx.grid_2d_graph(20, 30)

    # Ajout des arêtes en tenant compte des cases libres
    for row in range(20):
        for col in range(30):
            if grid[row, col] == 1:
                G.remove_node((row, col))

    # Calcul du chemin le plus court avec A*
    path = nx.astar_path(G, start, goal)

    # Détection des changements de direction (angles droits)
    directions = []

    for i in range(1, len(path) - 1):
        prev_node = path[i - 1]
        curr_node = path[i]
        next_node = path[i + 1]

        # Calcul de la direction entre les points consécutifs
        direction1 = (curr_node[0] - prev_node[0], curr_node[1] - prev_node[1])
        direction2 = (next_node[0] - curr_node[0], next_node[1] - curr_node[1])

        if direction1 != direction2:
            directions.append(curr_node)

    return path, directions
