from heapq import heappush, heappop  # Recommended.
import numpy as np
import heapq

from flightsim.world import World

def heuristic(a, b):

    """Computing the Euclidean distance heuristic for A*."""
    return np.linalg.norm(np.array(a) - np.array(b)) * 1.001  


def graph_search(world, resolution, margin, start, goal, astar):
    """
    Implements an optimized version of Dijkstra or A* for 3D path planning.

    Parameters:
        world      : World object representing the environment obstacles
        resolution : xyz resolution in meters for an occupancy map (shape=(3,))
        margin     : Minimum allowed distance in meters from path to obstacles.
        start      : xyz start position in meters (shape=(3,))
        goal       : xyz goal position in meters (shape=(3,))
        astar      : Boolean flag, if True use A*, else use Dijkstra.

    Returns:
        (path, nodes_expanded) where:
            path            : xyz positions (N,3), from start to goal (None if no path found).
            nodes_expanded  : Number of nodes expanded during search.
    """

    # Initialize occupancy map (discretized world representation)
    occ_map = world

    # Convert start and goal from metric space to voxel indices
    start_index = tuple(occ_map.metric_to_index(start))
    goal_index = tuple(occ_map.metric_to_index(goal))


    ###


    # If start or goal is inside an obstacle, returning no path
    if occ_map.is_occupied_index(start_index) or occ_map.is_occupied_index(goal_index):
        return None, 0

    # Priority queue min-heap for (cost, node)
    priority_queue = [(0, start_index)]
    g = {start_index: 0}  # Cost from start to each node
    came_from = {}

    # Tracking visited nodes
    visited = set()  

    # Allowing full 26-connected considering diagonal movements
    directions = [(dx, dy, dz)
                  for dx in [-1, 0, 1]
                  for dy in [-1, 0, 1]
                  for dz in [-1, 0, 1]
                  if not (dx == dy == dz == 0)] 
    

    # Number of nodes expanded
    nodes_expanded = 0

    while priority_queue:

        # Getting node with the lowest cost
        current_cost, u = heapq.heappop(priority_queue)

        # If already visited, skip 
        if u in visited:
            continue

        # Mark as visited
        visited.add(u)
        nodes_expanded += 1


        # Stopping early if goal is reached
        if u == goal_index:
            path = reconstruct_path(came_from, u, occ_map, start_index, start, goal)
            return path, nodes_expanded

        # Expanding neighbors
        for d in directions:
            v = (u[0] + d[0], u[1] + d[1], u[2] + d[2])

            # Skipping if out of bounds or occupied
            if not occ_map.is_valid_index(v) or occ_map.is_occupied_index(v):
                continue

            # Improved cost calculation
            move_cost = np.linalg.norm(d)  
            new_cost = g[u] + move_cost  

            # If new path to v is better, updating cost and pushing to queue
            if v not in g or new_cost < g[v]:
                g[v] = new_cost
                came_from[v] = u
                priority = new_cost + (heuristic(v, goal_index) if astar else 0)
                heapq.heappush(priority_queue, (priority, v))

    return None, nodes_expanded  


def reconstruct_path(came_from, current, occ_map, start_index, start, goal):

    """Backtracking from goal to start to reconstruct the path."""

    path = []

    while current in came_from:
        path.append(occ_map.index_to_metric_center(current))
        current = came_from[current]

    # Adding start position
    path.append(occ_map.index_to_metric_center(start_index))  

    # Reversing order (start to goal)
    path.reverse()  

    # Ensuring exact start and goal match
    path[0] = start
    path[-1] = goal

    return np.array(path)
