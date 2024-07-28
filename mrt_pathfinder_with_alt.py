import pandas as pd
import networkx as nx
import osmnx as ox
import folium
import heapq
import sys
from geopy.distance import geodesic

# Load the MRT stations and edges CSV files
stations_file_path = './MRT_Stations.csv'
edges_file_path = './MRT_Stations_Edges.csv'

mrt_stations = pd.read_csv(stations_file_path)
edges = pd.read_csv(edges_file_path)

# Create a new NetworkX graph
G = nx.Graph()

# Add nodes with coordinates for each MRT station according to CSV file
for idx, row in mrt_stations.iterrows():
    G.add_node(row['STN_NAME'], pos=(row['Longitude'], row['Latitude']))

# Add edges between the MRT stations according to CSV file
for idx, row in edges.iterrows():
    G.add_edge(row['Station'], row['Connected Station'])

# Input start and end MRT station from command-line arguments
if len(sys.argv) != 3:
    print("Usage: python mrt_pathfinder_with_alt.py <start_station> <end_station>")
    sys.exit(1)

start_station = sys.argv[1]
end_station = sys.argv[2]

# Emissions data in gCO2/km
emissions_data = {
    'mrt': 35  # Assuming 35 gCO2/km for MRT
}

# Generalized A* Algorithm
def heuristic(graph, node1, node2, is_osm=False):
    if is_osm:
        x1, y1 = graph.nodes[node1]['x'], graph.nodes[node1]['y']
        x2, y2 = graph.nodes[node2]['x'], graph.nodes[node2]['y']
    else:
        (x1, y1) = graph.nodes[node1]['pos']
        (x2, y2) = graph.nodes[node2]['pos']
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

def a_star(graph, start, end, is_osm=False):
    open_set = [(0, start)]
    g_costs = {node: float('infinity') for node in graph.nodes}
    f_costs = {node: float('infinity') for node in graph.nodes}
    previous_nodes = {node: None for node in graph.nodes}
    g_costs[start] = 0
    f_costs[start] = heuristic(graph, start, end, is_osm)

    while open_set:
        current_f_cost, current_node = heapq.heappop(open_set)

        if current_node == end:
            path = []
            while previous_nodes[current_node] is not None:
                path.append(current_node)
                current_node = previous_nodes[current_node]
            path.append(start)
            return path[::-1]

        for neighbor in graph.neighbors(current_node):
            if is_osm:
                weight = graph.edges[current_node, neighbor, 0].get('length', 1)
            else:
                weight = graph[current_node][neighbor].get('length', 1)
            tentative_g_cost = g_costs[current_node] + weight
            if tentative_g_cost < g_costs[neighbor]:
                previous_nodes[neighbor] = current_node
                g_costs[neighbor] = tentative_g_cost
                f_costs[neighbor] = tentative_g_cost + heuristic(graph, neighbor, end, is_osm)
                heapq.heappush(open_set, (f_costs[neighbor], neighbor))

    return []

# Function to calculate emissions based on distance
def calculate_emissions(distance_km, mode):
    return distance_km * emissions_data.get(mode, 0)

# Function to calculate the total distance of a path
def calculate_total_distance(graph, path, is_osm=False):
    total_distance = 0
    for i in range(len(path) - 1):
        if is_osm:
            distance = graph.edges[path[i], path[i + 1], 0].get('length', 1)
        else:
            node1_pos = graph.nodes[path[i]]['pos']
            node2_pos = graph.nodes[path[i + 1]]['pos']
            distance = geodesic((node1_pos[1], node1_pos[0]), (node2_pos[1], node2_pos[0])).meters
        total_distance += distance
    return total_distance

# K-Shortest Paths Algorithm
def yen_k_shortest_paths(graph, start, end, k=2):
    a_star_path = a_star(graph, start, end)
    if not a_star_path:
        return []

    paths = [a_star_path]
    potential_paths = []

    for i in range(1, k + 1):
        for j in range(len(paths[-1]) - 1):
            spur_node = paths[-1][j]
            root_path = paths[-1][:j + 1]

            removed_edges = []
            for path in paths:
                if len(path) > j and root_path == path[:j + 1]:
                    u = path[j]
                    v = path[j + 1]
                    if graph.has_edge(u, v):
                        graph.remove_edge(u, v)
                        removed_edges.append((u, v))

            visited = set(root_path)
            def custom_a_star(g, start, end, visited_nodes):
                open_set = [(0, start)]
                g_costs = {node: float('infinity') for node in g.nodes}
                f_costs = {node: float('infinity') for node in g.nodes}
                previous_nodes = {node: None for node in g.nodes}
                g_costs[start] = 0
                f_costs[start] = heuristic(g, start, end)

                while open_set:
                    current_f_cost, current_node = heapq.heappop(open_set)

                    if current_node == end:
                        path = []
                        while previous_nodes[current_node] is not None:
                            path.append(current_node)
                            current_node = previous_nodes[current_node]
                        path.append(start)
                        return path[::-1]

                    for neighbor in g.neighbors(current_node):
                        if neighbor in visited_nodes:
                            continue
                        weight = g[current_node][neighbor].get('length', 1)
                        tentative_g_cost = g_costs[current_node] + weight
                        if tentative_g_cost < g_costs[neighbor]:
                            previous_nodes[neighbor] = current_node
                            g_costs[neighbor] = tentative_g_cost
                            f_costs[neighbor] = tentative_g_cost + heuristic(g, neighbor, end)
                            heapq.heappush(open_set, (f_costs[neighbor], neighbor))

                return []

            spur_path = custom_a_star(graph, spur_node, end, visited)
            if spur_path:
                total_path = root_path[:-1] + spur_path
                if total_path not in paths and total_path not in [path[1] for path in potential_paths]:
                    potential_paths.append((len(total_path), total_path))

            for u, v in removed_edges:
                graph.add_edge(u, v)

        if not potential_paths:
            break

        potential_paths.sort()
        paths.append(potential_paths.pop(0)[1])

    return paths

# Use the generalized A* function to find the shortest path in MRT network
a_star_shortest_path = a_star(G, start_station, end_station)
if not a_star_shortest_path:
    print(f"No path found between {start_station} and {end_station}")
    sys.exit(1)

# Calculate distance and emissions for the shortest path
shortest_distance_km = calculate_total_distance(G, a_star_shortest_path) / 1000  # Convert to kilometers
shortest_emissions = calculate_emissions(shortest_distance_km, 'mrt')
print(f"A* MRT Route Emissions: {shortest_emissions} gCO2")

# Get 2 alternate paths
k_shortest_paths = yen_k_shortest_paths(G, start_station, end_station, k=2)

# Calculate distance and emissions for the alternate paths
for i, path in enumerate(k_shortest_paths):
    distance_km = calculate_total_distance(G, path) / 1000  # Convert to kilometers
    emissions = calculate_emissions(distance_km, 'mrt')
    print(f"Alternate {i + 1} MRT Route Emissions: {emissions} gCO2")

# Download the street network for Singapore using osmnx
place_name = "Singapore"
# graph = ox.graph_from_place(place_name, network_type='all', truncate_by_edge=True, simplify=True)

# local graph
graph = ox.graph_from_xml("./singapore.osm", simplify=False)

# Find the nearest nodes in the OSM network for each MRT station
nearest_nodes = {}
for idx, row in mrt_stations.iterrows():
    nearest_node = ox.distance.nearest_nodes(graph, X=row['Longitude'], Y=row['Latitude'])
    nearest_nodes[row['STN_NAME']] = nearest_node

# Convert MRT shortest path to OSM shortest path for plotting
def convert_to_osm_path(shortest_path, algorithm):
    osm_path = []
    for i in range(len(shortest_path) - 1):
        start_node = nearest_nodes[shortest_path[i]]
        end_node = nearest_nodes[shortest_path[i + 1]]
        path = a_star(graph, start_node, end_node, is_osm=True)
        if not path:
            print(f"No path between {shortest_path[i]} and {shortest_path[i + 1]}")
            osm_path = []
            break
        osm_path.extend(path)
    return osm_path

# Convert paths to OSM paths using A* algorithm
osm_paths = [convert_to_osm_path(path, 'a_star') for path in k_shortest_paths]

# Create maps centered around Singapore
map_1 = folium.Map(location=[1.3521, 103.8198], zoom_start=12)
map_2 = folium.Map(location=[1.3521, 103.8198], zoom_start=12)
map_3 = folium.Map(location=[1.3521, 103.8198], zoom_start=12)

colors = ['red', 'blue', 'green'] # Colors for each route

# Plot routes on the maps
for osm_path, color, map_sg, route in zip(osm_paths, colors, [map_1, map_2, map_3], k_shortest_paths):
    if osm_path:
        path_coords = [(graph.nodes[node]['y'], graph.nodes[node]['x']) for node in osm_path]
        folium.PolyLine(locations=path_coords, color=color, weight=5, tooltip=f'Path {colors.index(color) + 1}').add_to(map_sg)
        # Add markers to the MRT stations along the plotted route
        for station in route:
            row = mrt_stations[mrt_stations['STN_NAME'] == station].iloc[0]
            if station == start_station:
                folium.Marker(location=[row['Latitude'], row['Longitude']], popup=row['STN_NAME'], icon=folium.Icon(color='green')).add_to(map_sg)
            elif station == end_station:
                folium.Marker(location=[row['Latitude'], row['Longitude']], popup=row['STN_NAME'], icon=folium.Icon(color='red')).add_to(map_sg)
            else:
                folium.Marker(location=[row['Latitude'], row['Longitude']], popup=row['STN_NAME']).add_to(map_sg)

# Save and display the maps
map_1.save('mrt_route_1.html')
map_2.save('mrt_route_2.html')
map_3.save('mrt_route_3.html')

# Print the paths to the generated maps and the routes
print("Route 1:", a_star_shortest_path)
for i, path in enumerate(k_shortest_paths[1:], start=2):
    print(f"Route {i}:", path)
print('mrt_route_1.html')
print('mrt_route_2.html')
print('mrt_route_3.html')