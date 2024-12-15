import numpy as np
import networkx as nx
import math

def world_to_grid(xy, origin, resolution):
    # Converts world coordinates in meters to grid indices
    xy = np.reshape(xy, (-1,2))
    origin = np.reshape(origin, (-1,2))
    
    if xy.size == 0:
        ij = xy
    else:
        ij = np.floor((xy - origin) / resolution)
    ij = ij[:, [1, 0]]  # Convert from (columns,rows) to (rows,columns)
    return np.int64(ij)

def grid_to_world(ij, origin, resolution):
    # Converts grid indices to world coordinates in meters
    ij = np.reshape(ij, (-1,2))
    ij = ij[:, [1, 0]]  # Convert from (rows,columns) to (columns,rows)
    origin = np.reshape(origin, (-1,2))

    if ij.size == 0:
        xy = np.zeros((0,2))
    else:
        xy = ij * resolution + (origin + 0.5 * resolution)

    return xy

def costmap_connectivity(costmap_matrix, connectivity_threshold=0.0, diagonal_connectivity=False, edge_weight='mean'):
    """
    Build adjacency (edges) from the costmap by checking neighbors.
    The returned edge_list contains pairs of flattened node indices,
    and weight_list contains edge weights.
    """
    if edge_weight.lower() == 'mean':
        edge_weight_func = lambda a, b: (a + b) / 2.0
    elif edge_weight.lower() == 'sum':
        edge_weight_func = lambda a, b: (a + b)
    elif edge_weight.lower() == 'min': 
        edge_weight_func = lambda a, b: np.minimum(a, b)
    elif edge_weight.lower() == 'max':
        edge_weight_func = lambda a, b: np.maximum(a, b)
    else:
        raise ValueError('Unknown edge weight type')

    costmap_matrix = np.array(costmap_matrix)
    
    neighbor_offsets = [(-1, 0), (+1, 0), (0, -1), (0, +1)]
    if diagonal_connectivity:
        neighbor_offsets.extend([(-1, -1), (-1, +1), (+1, -1), (+1, +1)])
    
    rows, cols = costmap_matrix.shape
    edge_list = []
    weight_list = []

    for drow, dcol in neighbor_offsets:
        start_r_indices, start_c_indices = np.meshgrid(
            np.arange(max(0, -drow), min(rows, rows - drow)),
            np.arange(max(0, -dcol), min(cols, cols - dcol))
        )
        end_r_indices, end_c_indices = np.meshgrid(
            np.arange(max(0, drow), min(rows, rows + drow)),
            np.arange(max(0, dcol), min(cols, cols + dcol))
        )

        start_flat = np.ravel_multi_index((start_r_indices.flatten(), start_c_indices.flatten()), (rows, cols))
        end_flat = np.ravel_multi_index((end_r_indices.flatten(), end_c_indices.flatten()), (rows, cols))

        start_cost = costmap_matrix[start_r_indices, start_c_indices].flatten()
        end_cost = costmap_matrix[end_r_indices, end_c_indices].flatten()

        if drow * dcol == 0.0:
            weights = edge_weight_func(start_cost, end_cost)
        else:
            # Diagonal edges might have sqrt(2) factor
            weights = edge_weight_func(start_cost, end_cost) * np.sqrt(2)

        # Both start and end cells must be traversable (cost > 0)
        valid = np.logical_and(start_cost > 0, end_cost > 0)
        edge_list.extend(zip(start_flat[valid], end_flat[valid]))
        weight_list.extend(weights[valid])

    return edge_list, weight_list

def costmap_graph_networkx(costmap_matrix, diagonal_connectivity=False):
    """
    Construct a weighted undirected graph from the costmap.
    Nodes are flattened indices of each grid cell.
    Edges are between neighboring traversable cells.
    """
    edge_list, weight_list = costmap_connectivity(
        costmap_matrix=costmap_matrix, 
        diagonal_connectivity=diagonal_connectivity
    )
    weighted_edges = [(e[0], e[1], w) for (e, w) in zip(edge_list, weight_list)]
    graph = nx.Graph()
    graph.add_weighted_edges_from(weighted_edges, weight='weight')
    return graph

def shortest_path_networkx(costmap_matrix, source_cell, target_cell, diagonal_connectivity=False):
    """
    Compute the shortest path using A* instead of Dijkstra.
    """

    graph = costmap_graph_networkx(costmap_matrix, diagonal_connectivity=diagonal_connectivity)
    source_node = np.ravel_multi_index(source_cell, dims=costmap_matrix.shape)
    target_node = np.ravel_multi_index(target_cell, dims=costmap_matrix.shape)

    # Define a Euclidean heuristic for A*
    def euclidean_heuristic(u, v):
        rows, cols = costmap_matrix.shape
        ru, cu = np.unravel_index(u, (rows, cols))
        rv, cv = np.unravel_index(v, (rows, cols))
        return np.sqrt((ru - rv)**2 + (cu - cv)**2)

    try:
        # Use A* path planner with a Euclidean heuristic
        path = nx.astar_path(graph, source_node, target_node, heuristic=euclidean_heuristic, weight='weight')
        path = np.unravel_index(path, costmap_matrix.shape)
        path = np.column_stack((path[0], path[1]))  # shape: (N, 2), each row = (row, col)
    except nx.exception.NetworkXNoPath:
        path = np.zeros((0,2))

    return path
