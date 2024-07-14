import osmnx as ox
import folium
import heapq
import random
import sys

# Dijkstra's Algorithm Function
def dijkstra(graph, start, end, penalties=None):
    if penalties is None:
        penalties = {}
    queue = [(0, start)]
    distances = {node: float('infinity') for node in graph.nodes}
    previous_nodes = {node: None for node in graph.nodes}
    distances[start] = 0

    while queue:
        current_distance, current_node = heapq.heappop(queue)

        if current_node == end:
            path = []
            while previous_nodes[current_node] is not None:
                path.append(current_node)
                current_node = previous_nodes[current_node]
            path.append(start)
            return path[::-1], distances[end]

        if current_distance > distances[current_node]:
            continue

        for neighbor in graph.neighbors(current_node):
            weight = graph.edges[current_node, neighbor, 0].get('length', 1)
            if (current_node, neighbor) in penalties:
                weight += penalties[(current_node, neighbor)]
            distance = current_distance + weight

            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous_nodes[neighbor] = current_node
                heapq.heappush(queue, (distance, neighbor))

    return [], float('infinity')

# A* Algorithm Function
def heuristic(graph, node1, node2):
    x1, y1 = graph.nodes[node1]['x'], graph.nodes[node1]['y']
    x2, y2 = graph.nodes[node2]['x'], graph.nodes[node2]['y']
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

def a_star(graph, start, end, penalties=None):
    if penalties is None:
        penalties = {}
    open_set = [(0, start)]
    g_costs = {node: float('infinity') for node in graph.nodes}
    f_costs = {node: float('infinity') for node in graph.nodes}
    previous_nodes = {node: None for node in graph.nodes}
    g_costs[start] = 0
    f_costs[start] = heuristic(graph, start, end)

    while open_set:
        current_f_cost, current_node = heapq.heappop(open_set)

        if current_node == end:
            path = []
            while previous_nodes[current_node] is not None:
                path.append(current_node)
                current_node = previous_nodes[current_node]
            path.append(start)
            return path[::-1], g_costs[end]

        for neighbor in graph.neighbors(current_node):
            weight = graph.edges[current_node, neighbor, 0].get('length', 1)
            if (current_node, neighbor) in penalties:
                weight += penalties[(current_node, neighbor)]
            tentative_g_cost = g_costs[current_node] + weight
            if tentative_g_cost < g_costs[neighbor]:
                previous_nodes[neighbor] = current_node
                g_costs[neighbor] = tentative_g_cost
                f_costs[neighbor] = tentative_g_cost + heuristic(graph, neighbor, end)
                heapq.heappush(open_set, (f_costs[neighbor], neighbor))

    return [], float('infinity')

# Travelling Salesman Problem Algorithm using Nearest Neighbor Heuristic (Greedy)
def greedy_tsp(graph, nodes):
    start_node = nodes[0]
    end_node = nodes[-1]
    tsp_path = [start_node]
    unvisited = set(nodes[1:])

    while unvisited:
        last_node = tsp_path[-1]
        if last_node == end_node:
            break
        next_node = min(unvisited, key=lambda node: dijkstra(graph, last_node, node)[1])
        tsp_path.append(next_node)
        unvisited.remove(next_node)

    if tsp_path[-1] != end_node:
        tsp_path.append(end_node)

    return tsp_path

def calculate_total_distance(graph, path):
    total_distance = 0
    for i in range(len(path) - 1):
        _, distance = dijkstra(graph, path[i], path[i + 1])
        total_distance += distance
    return total_distance

# 2-opt Algorithm to improve TSP route
def two_opt(graph, path):
    best_path = path
    best_distance = calculate_total_distance(graph, best_path)
    
    improved = True
    while improved:
        improved = False
        for i in range(1, len(path) - 2):
            for j in range(i + 1, len(path) - 1):
                if j - i == 1: continue  # Skip consecutive nodes

                new_path = path[:i] + path[i:j][::-1] + path[j:]
                new_distance = calculate_total_distance(graph, new_path)
                
                if new_distance < best_distance:
                    best_path = new_path
                    best_distance = new_distance
                    improved = True
        path = best_path
    return best_path

def get_edges_from_path(path):
    return [(path[i], path[i + 1]) for i in range(len(path) - 1)]

# Function to apply penalty to prevent overlapping routes
def apply_penalties_to_graph(graph, path, penalty_value):
    penalties = {}
    edges = get_edges_from_path(path)
    for edge in edges:
        penalties[edge] = penalty_value
    return penalties

# Function to calculate full route for TSP
def calculate_full_tsp_route(graph, tsp_path, penalties=None):
    full_path = []
    for i in range(len(tsp_path) - 1):
        segment_path, _ = dijkstra(graph, tsp_path[i], tsp_path[i + 1], penalties=penalties)
        full_path.extend(segment_path[:-1])
    full_path.append(tsp_path[-1])
    return full_path

# Individually define penalty for each transport mode. Increase if routes are overlapping too often
penalty_factors = {
    'drive': 1.8,
    'bike': 2.6,
    'walk': 3.4
}

# Individually define number of intermediate nodes for each transport mode
intermediate_nodes_count = {
    'drive': 3,
    'bike': 2,
    'walk': 2
}

# Map transport modes to their corresponding saved maps
transport_modes = ['drive', 'bike', 'walk']

# Input start and end coordinates from command-line arguments
if len(sys.argv) != 5:
    print("Usage: python drive_bike_walk_pathfinder.py <start_lat> <start_long> <end_lat> <end_long>")
    sys.exit(1)

start_coords = (float(sys.argv[1]), float(sys.argv[2]))
end_coords = (float(sys.argv[3]), float(sys.argv[4]))

# Generate routes for each transport mode
for mode in transport_modes:
    graph = ox.graph_from_place("Singapore", network_type=mode, truncate_by_edge=False, simplify=False)

    start_node = ox.distance.nearest_nodes(graph, X=start_coords[1], Y=start_coords[0])
    end_node = ox.distance.nearest_nodes(graph, X=end_coords[1], Y=end_coords[0])

    # Calculate original paths using A* algorithm
    a_star_path, a_star_distance = a_star(graph, start_node, end_node)

    # Generate random intermediate points based on transport mode
    num_intermediate_nodes = intermediate_nodes_count[mode]
    intermediate_nodes = random.sample(list(graph.nodes), num_intermediate_nodes)
    tsp_nodes = [start_node] + intermediate_nodes + [end_node]

    # Use TSP with 2-opt to plot alternate routes
    tsp_path = two_opt(graph, greedy_tsp(graph, tsp_nodes))
    tsp_full_path = calculate_full_tsp_route(graph, tsp_path)

    # Apply Penalties
    penalty_factor = penalty_factors.get(mode, 1.5) # if not defined then set penalty as 1.5
    penalties = apply_penalties_to_graph(graph, tsp_full_path, penalty_factor)

    # Solve TSP for second route
    tsp_path_alternate = two_opt(graph, greedy_tsp(graph, tsp_nodes))
    tsp_full_path_alternate = calculate_full_tsp_route(graph, tsp_path_alternate, penalties=penalties)

    # Plot each route on separate maps
    for idx, (path, color, label) in enumerate([
        (a_star_path, 'red', f'A* {mode.capitalize()} Route'),
        (tsp_full_path, 'blue', f'TSP {mode.capitalize()} Route'),
        (tsp_full_path_alternate, 'green', f'Alternate TSP {mode.capitalize()} Route')
    ], start=1):
        map_sg = folium.Map(location=[1.3521, 103.8198], zoom_start=12)

        if path:
            path_coords = [(graph.nodes[node]['y'], graph.nodes[node]['x']) for node in path]
            folium.PolyLine(locations=path_coords, color=color, weight=5, tooltip=label).add_to(map_sg)

        # Add markers for start and end points
        folium.Marker(location=[start_coords[0], start_coords[1]], popup='Start', icon=folium.Icon(color='green')).add_to(map_sg)
        folium.Marker(location=[end_coords[0], end_coords[1]], popup='End', icon=folium.Icon(color='red')).add_to(map_sg)

        # Save the map
        map_sg.save(f'{mode}_route_{idx}.html')

# Print the paths to the generated maps
for mode in transport_modes:
    for i in range(1, 4):
        print(f'{mode}_route_{i}.html')
