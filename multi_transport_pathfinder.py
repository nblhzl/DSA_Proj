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

# Input start and end coordinates from command-line arguments
if len(sys.argv) != 5:
    print("Usage: python multi_transport_pathfinder.py <start_lat> <start_long> <end_lat> <end_long>")
    sys.exit(1)

# Emissions data in gCO2/km
emissions_data = {
    'car': 170,
    'bike': 0,  # Assuming negligible emissions
    'walk': 0,  # Assuming negligible emissions
    'mrt': 35  # Assuming 35 gCO2/km for MRT
}

start_coords = (float(sys.argv[1]), float(sys.argv[2]))
end_coords = (float(sys.argv[3]), float(sys.argv[4]))

# Function to calculate emissions based on distance
def calculate_emissions(distance_km, mode):
    return distance_km * emissions_data.get(mode, 0)

# Generalized A* Algorithm Function
def heuristic(graph, node1, node2, coord_attr='pos'):
    if coord_attr == 'pos':
        (x1, y1) = graph.nodes[node1][coord_attr]
        (x2, y2) = graph.nodes[node2][coord_attr]
    else:
        x1, y1 = graph.nodes[node1]['x'], graph.nodes[node1]['y']
        x2, y2 = graph.nodes[node2]['x'], graph.nodes[node2]['y']
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

def a_star(graph, start, end, coord_attr='pos', is_osm=False):
    open_set = [(0, start)]
    g_costs = {node: float('infinity') for node in graph.nodes}
    f_costs = {node: float('infinity') for node in graph.nodes}
    previous_nodes = {node: None for node in graph.nodes}
    g_costs[start] = 0
    f_costs[start] = heuristic(graph, start, end, coord_attr)

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
            if is_osm:
                weight = graph.edges[current_node, neighbor, 0].get('length', 1)
            else:
                weight = graph[current_node][neighbor].get('length', 1)
            tentative_g_cost = g_costs[current_node] + weight
            if tentative_g_cost < g_costs[neighbor]:
                previous_nodes[neighbor] = current_node
                g_costs[neighbor] = tentative_g_cost
                f_costs[neighbor] = tentative_g_cost + heuristic(graph, neighbor, end, coord_attr)
                heapq.heappush(open_set, (f_costs[neighbor], neighbor))

    return [], float('infinity')

# Download Singapore street network using osmnx
place_name = "Singapore"
# graph = ox.graph_from_place(place_name, network_type='all', truncate_by_edge=True, simplify=True)

# local graph
graph = ox.graph_from_xml("./singapore.osm", simplify=False)

# Find the nearest nodes in the OSM network for each MRT station
nearest_nodes = {}
for idx, row in mrt_stations.iterrows():
    nearest_node = ox.distance.nearest_nodes(graph, X=row['Longitude'], Y=row['Latitude'])
    nearest_nodes[row['STN_NAME']] = nearest_node

# Find the nearest nodes in the OSM network for start and end coordinates
start_node = ox.distance.nearest_nodes(graph, X=start_coords[1], Y=start_coords[0])
end_node = ox.distance.nearest_nodes(graph, X=end_coords[1], Y=end_coords[0])

# Function to find the nearest MRT stations
def find_nearest_stations(coords, mrt_stations, n=3):
    distances = mrt_stations.apply(lambda row: ox.distance.euclidean(coords[0], coords[1], row['Latitude'], row['Longitude']), axis=1)
    nearest_indices = distances.nsmallest(n).index
    return mrt_stations.loc[nearest_indices, 'STN_NAME'].values

# Find the nearest, second nearest, and third nearest MRT stations to the start and end coordinates
nearest_start_stations = find_nearest_stations(start_coords, mrt_stations)
nearest_end_stations = find_nearest_stations(end_coords, mrt_stations)

# Function to compute heuristic distances for start and end point to nearest mrt stations
def compute_heuristic_distance(start_coords, end_coords, start_station, end_station, mrt_stations):
    start_station_coords = (mrt_stations.loc[mrt_stations['STN_NAME'] == start_station, 'Latitude'].values[0],
                            mrt_stations.loc[mrt_stations['STN_NAME'] == start_station, 'Longitude'].values[0])
    end_station_coords = (mrt_stations.loc[mrt_stations['STN_NAME'] == end_station, 'Latitude'].values[0],
                          mrt_stations.loc[mrt_stations['STN_NAME'] == end_station, 'Longitude'].values[0])
    start_to_station = ox.distance.euclidean(start_coords[0], start_coords[1], start_station_coords[0], start_station_coords[1])
    station_to_end = ox.distance.euclidean(end_coords[0], end_coords[1], end_station_coords[0], end_station_coords[1])
    return start_to_station + station_to_end

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

# Generate combinations and their heuristic distances
combinations = []
for start_station in nearest_start_stations:
    for end_station in nearest_end_stations:
        distance = compute_heuristic_distance(start_coords, end_coords, start_station, end_station, mrt_stations)
        combinations.append((start_station, end_station, distance))

# Sort combinations by heuristic distance
combinations.sort(key=lambda x: x[2])

# Select top 3 combinations
top_combinations = combinations[:3]

print("Top combinations based on heuristic distance:")
for comb in top_combinations:
    print(f"Start: {comb[0]}, End: {comb[1]}, Distance: {comb[2]}")

# Use A* algorithm to find the shortest paths in MRT network for top combinations
a_star_paths = []
for start_station, end_station, _ in top_combinations:
    path, _ = a_star(G, start_station, end_station)
    if path:
        a_star_paths.append(path)

# Print all paths found
for i, path in enumerate(a_star_paths):
    print(f"A* MRT path {i+1}:", path)

# Calculate distance and emissions for the MRT paths
for i, path in enumerate(a_star_paths):
    distance_km = calculate_total_distance(G, path) / 1000  # Convert to kilometers
    emissions = calculate_emissions(distance_km, 'mrt')
    print(f"A* MRT Route {i+1} Emissions: {emissions} gCO2")

# Convert MRT shortest paths to OSM shortest paths for plotting
def convert_to_osm_path(shortest_path):
    osm_path = []
    for i in range(len(shortest_path) - 1):
        start_node = nearest_nodes[shortest_path[i]]
        end_node = nearest_nodes[shortest_path[i + 1]]
        path, _ = a_star(graph, start_node, end_node, coord_attr='x', is_osm=True)
        if not path:
            print(f"No path between {shortest_path[i]} and {shortest_path[i + 1]}")
            osm_path = []
            break
        osm_path.extend(path)
    return osm_path

osm_paths = []
for path in a_star_paths:
    osm_path = convert_to_osm_path(path)
    if osm_path:
        osm_paths.append(osm_path)

# Function to plot paths
def plot_paths(map_obj, start_to_mrt_path, mrt_to_end_path, mrt_path, start_to_mrt_distance, mrt_to_end_distance, mrt_color, algorithm_name):
    # Add markers to MRT stations
    for idx, row in mrt_stations.iterrows():
        folium.Marker(location=[row['Latitude'], row['Longitude']], popup=row['STN_NAME']).add_to(map_obj)

    # Plot walking/biking path from start coordinate to nearest MRT station
    if start_to_mrt_path:
        start_to_mrt_coords = [(graph.nodes[node]['y'], graph.nodes[node]['x']) for node in start_to_mrt_path]
        print("Distance from Start coords to MRT station:", start_to_mrt_distance)
        if start_to_mrt_distance <= 800: # plot a walk route if distance is less than 800m
            folium.PolyLine(locations=start_to_mrt_coords, color='green', weight=5, tooltip=f'Walking Start to MRT ({algorithm_name})').add_to(map_obj)
        else:
            folium.PolyLine(locations=start_to_mrt_coords, color='purple', weight=5, tooltip=f'Biking Start to MRT ({algorithm_name})').add_to(map_obj)

    # Plot walking/biking path from nearest MRT station to end coordinate
    if mrt_to_end_path:
        mrt_to_end_coords = [(graph.nodes[node]['y'], graph.nodes[node]['x']) for node in mrt_to_end_path]
        print("Distance from MRT station to end coords:", mrt_to_end_distance)
        if mrt_to_end_distance <= 800: # plot a walk route if distance is less than 800m
            folium.PolyLine(locations=mrt_to_end_coords, color='green', weight=5, tooltip=f'Walking MRT to End ({algorithm_name})').add_to(map_obj)
        else:
            folium.PolyLine(locations=mrt_to_end_coords, color='purple', weight=5, tooltip=f'Biking MRT to End ({algorithm_name})').add_to(map_obj)

    # Plot the MRT path
    if mrt_path:
        mrt_path_coords = [(graph.nodes[node]['y'], graph.nodes[node]['x']) for node in mrt_path]
        folium.PolyLine(locations=mrt_path_coords, color=mrt_color, weight=5, tooltip=f'MRT Path ({algorithm_name})').add_to(map_obj)

    # Add markers for start and end points
    folium.Marker(location=[start_coords[0], start_coords[1]], popup='Start', icon=folium.Icon(color='green')).add_to(map_obj)
    folium.Marker(location=[end_coords[0], end_coords[1]], popup='End', icon=folium.Icon(color='red')).add_to(map_obj)

# Plot paths on maps
maps = []
colors = ['red', 'blue', 'green'] # Colors for mrt routes
for i, osm_path in enumerate(osm_paths):
    map_obj = folium.Map(location=[1.3521, 103.8198], zoom_start=12)
    start_station = top_combinations[i][0]
    end_station = top_combinations[i][1]
    start_to_mrt_path, start_to_mrt_distance = a_star(graph, start_node, nearest_nodes[start_station], coord_attr='x', is_osm=True)
    mrt_to_end_path, mrt_to_end_distance = a_star(graph, nearest_nodes[end_station], end_node, coord_attr='x', is_osm=True)
    plot_paths(map_obj, start_to_mrt_path, mrt_to_end_path, osm_path, start_to_mrt_distance, mrt_to_end_distance, colors[i % len(colors)], f'A* Path {i+1}')
    maps.append(map_obj)

# Save and display the maps
for i, map_obj in enumerate(maps):
    map_obj.save(f'multi_transport_{i+1}.html')

# Print the path to the generated map
print('multi_transport_1.html')
print('multi_transport_2.html')
print('multi_transport_3.html')