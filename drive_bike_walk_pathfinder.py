import osmnx as ox
import folium
import heapq
import networkx as nx
from geopy.distance import geodesic

# Emissions in kg CO2 per km
EMISSIONS_FACTORS = {
    'drive': 0.170,  # Petrol car
    'bike': 0.0,  # zero emissions for bikes
    'walk': 0.0  # zero emissions for walking
}

# Generalized Dijkstra's Algorithm Function
def dijkstra(graph, start, end):
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
            distance = current_distance + weight

            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous_nodes[neighbor] = current_node
                heapq.heappush(queue, (distance, neighbor))

    return [], float('infinity')

# Generalized A* Algorithm Function
def heuristic(graph, node1, node2):
    x1, y1 = graph.nodes[node1]['x'], graph.nodes[node1]['y']
    x2, y2 = graph.nodes[node2]['x'], graph.nodes[node2]['y']
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

def a_star(graph, start, end):
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
            tentative_g_cost = g_costs[current_node] + weight
            if tentative_g_cost < g_costs[neighbor]:
                previous_nodes[neighbor] = current_node
                g_costs[neighbor] = tentative_g_cost
                f_costs[neighbor] = tentative_g_cost + heuristic(graph, neighbor, end)
                heapq.heappush(open_set, (f_costs[neighbor], neighbor))

    return [], float('infinity')

def calculate_distance(path, graph):
    total_distance = 0.0
    for i in range(len(path) - 1):
        pos1 = (graph.nodes[path[i]]['y'], graph.nodes[path[i]]['x'])
        pos2 = (graph.nodes[path[i + 1]]['y'], graph.nodes[path[i + 1]]['x'])
        total_distance += geodesic(pos1, pos2).km
    return total_distance

def generate_and_display_paths_drive_bike_walk(start_coords, end_coords):
    # Map transport modes to their corresponding saved maps
    transport_modes = {
        'drive': 'drive_route_map.html',
        'bike': 'bike_route_map.html',
        'walk': 'walk_route_map.html'
    }

    paths_info = {}

    # Generate routes for each transport mode
    for mode, output_file in transport_modes.items():
        graph = ox.graph_from_place("Singapore", network_type=mode, truncate_by_edge=False, simplify=False)

        start_node = ox.distance.nearest_nodes(graph, X=start_coords[1], Y=start_coords[0])
        end_node = ox.distance.nearest_nodes(graph, X=end_coords[1], Y=end_coords[0])

        # Calculate paths using Dijkstra and A* algorithm
        dijkstra_path, _ = dijkstra(graph, start_node, end_node)
        a_star_path, _ = a_star(graph, start_node, end_node)

        # Calculate distances
        dijkstra_distance = calculate_distance(dijkstra_path, graph)
        a_star_distance = calculate_distance(a_star_path, graph)

        # Calculate emissions
        dijkstra_emissions = dijkstra_distance * EMISSIONS_FACTORS[mode]
        a_star_emissions = a_star_distance * EMISSIONS_FACTORS[mode]

        paths_info[mode] = {
            'dijkstra': {
                'path': dijkstra_path,
                'distance': dijkstra_distance,  
                'emissions': dijkstra_emissions  
            },
            'a_star': {
                'path': a_star_path,
                'distance': a_star_distance,  
                'emissions': a_star_emissions  
            },
            'file': output_file
        }

        # Create a folium map centered around Singapore
        map_sg = folium.Map(location=[1.3521, 103.8198], zoom_start=12)

        # Plot Dijkstra route
        if dijkstra_path:
            dijkstra_path_coords = [(graph.nodes[node]['y'], graph.nodes[node]['x']) for node in dijkstra_path]
            folium.PolyLine(locations=dijkstra_path_coords, color='blue', weight=5, tooltip=f'Dijkstra {mode.capitalize()} Route').add_to(map_sg)

        # Plot A* route
        if a_star_path:
            a_star_path_coords = [(graph.nodes[node]['y'], graph.nodes[node]['x']) for node in a_star_path]
            folium.PolyLine(locations=a_star_path_coords, color='red', weight=5, tooltip=f'A* {mode.capitalize()} Route').add_to(map_sg)

        # Save the map
        map_sg.save(output_file)

    return paths_info

if __name__ == "__main__":
    # For testing purposes
    start_coords = (1.390505, 103.986793)
    end_coords = (1.379192, 103.759387)
    generate_and_display_paths_drive_bike_walk(start_coords, end_coords)
