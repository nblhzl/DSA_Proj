import pandas as pd
import networkx as nx
import osmnx as ox
import folium
import heapq
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

# Emissions factors in kg CO2 per km
MRT_EMISSIONS_FACTOR = 0.028  

# Generalized Dijkstra's Algorithm Function
def dijkstra(graph, start, end, is_osm=False):
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
            path_distance = distances[end]
            return path[::-1], path_distance

        if current_distance > distances[current_node]:
            continue

        for neighbor in graph.neighbors(current_node):
            if is_osm:
                weight = graph.edges[current_node, neighbor, 0].get('length', 1)  # graph.edges to prevent unnecessary paths taken
            else:
                weight = graph[current_node][neighbor].get('length', 1)
            distance = current_distance + weight

            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous_nodes[neighbor] = current_node
                heapq.heappush(queue, (distance, neighbor))

    return [], float('infinity')

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
            path_distance = g_costs[end]
            return path[::-1], path_distance

        for neighbor in graph.neighbors(current_node):
            if is_osm:
                weight = graph.edges[current_node, neighbor, 0].get('length', 1)  # graph.edges to prevent unnecessary paths taken
            else:
                weight = graph[current_node][neighbor].get('length', 1)
            tentative_g_cost = g_costs[current_node] + weight
            if tentative_g_cost < g_costs[neighbor]:
                previous_nodes[neighbor] = current_node
                g_costs[neighbor] = tentative_g_cost
                f_costs[neighbor] = tentative_g_cost + heuristic(graph, neighbor, end, coord_attr)
                heapq.heappush(open_set, (f_costs[neighbor], neighbor))

    return [], float('infinity')

def calculate_distance(path, graph):
    total_distance = 0.0
    for i in range(len(path) - 1):
        pos1 = graph.nodes[path[i]]['pos']
        pos2 = graph.nodes[path[i + 1]]['pos']
        total_distance += geodesic((pos1[1], pos1[0]), (pos2[1], pos2[0])).km
    return total_distance

def find_nearest_station(coords):
    nearest_station = None
    shortest_distance = float('inf')
    for idx, row in mrt_stations.iterrows():
        station_coords = (row['Latitude'], row['Longitude'])
        distance = geodesic(coords, station_coords).km
        if distance < shortest_distance:
            nearest_station = row['STN_NAME']
            shortest_distance = distance
    return nearest_station

def generate_and_display_paths(start, end):
    # Determine if start and end are coordinates or station names
    try:
        start_coords = tuple(map(float, start.split(',')))
        start_station = find_nearest_station(start_coords)
    except ValueError:
        start_station = start

    try:
        end_coords = tuple(map(float, end.split(',')))
        end_station = find_nearest_station(end_coords)
    except ValueError:
        end_station = end

    # Use the generalized Dijkstra function to find the shortest path in MRT network
    dijkstra_shortest_path, _ = dijkstra(G, start_station, end_station)
    dijkstra_distance = calculate_distance(dijkstra_shortest_path, G)
    dijkstra_emissions = dijkstra_distance * MRT_EMISSIONS_FACTOR
    dijkstra_stops = len(dijkstra_shortest_path) - 1
    print("Dijkstra shortest path:", dijkstra_shortest_path)
    print("Dijkstra distance (km):", dijkstra_distance)
    print("Dijkstra emissions (kg CO2):", dijkstra_emissions)
    print("Dijkstra stops:", dijkstra_stops)

    # Use the generalized A* function to find the shortest path in MRT network
    a_star_shortest_path, _ = a_star(G, start_station, end_station)
    a_star_distance = calculate_distance(a_star_shortest_path, G)
    a_star_emissions = a_star_distance * MRT_EMISSIONS_FACTOR
    a_star_stops = len(a_star_shortest_path) - 1
    print("A* shortest path:", a_star_shortest_path)
    print("A* distance (km):", a_star_distance)
    print("A* emissions (kg CO2):", a_star_emissions)
    print("A* stops:", a_star_stops)

    # Download the street network for Singapore using osmnx
    place_name = "Singapore"
    graph = ox.graph_from_place(place_name, network_type='all', simplify=True)

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
            if algorithm == 'dijkstra':
                path, _ = dijkstra(graph, start_node, end_node, is_osm=True)
            elif algorithm == 'a_star':
                path, _ = a_star(graph, start_node, end_node, coord_attr='x', is_osm=True)
            else:
                raise ValueError("Invalid algorithm specified. Use 'dijkstra' or 'a_star'.")
            if not path:
                print(f"No path between {shortest_path[i]} and {shortest_path[i + 1]}")
                osm_path = []
                break
            osm_path.extend(path)
        return osm_path

    # Convert paths to OSM paths using Dijkstra and A* algorithms
    osm_dijkstra_path = convert_to_osm_path(dijkstra_shortest_path, 'dijkstra')
    osm_a_star_path = convert_to_osm_path(a_star_shortest_path, 'a_star')

    # Create a folium map centered around Singapore
    map_sg = folium.Map(location=[1.3521, 103.8198], zoom_start=12)

    # Add markers to MRT stations
    for idx, row in mrt_stations.iterrows():
        folium.Marker(location=[row['Latitude'], row['Longitude']], popup=row['STN_NAME']).add_to(map_sg)

    # Plot Dijkstra route on the map
    if osm_dijkstra_path:
        dijkstra_path_coords = [(graph.nodes[node]['y'], graph.nodes[node]['x']) for node in osm_dijkstra_path]
        folium.PolyLine(locations=dijkstra_path_coords, color='blue', weight=5, tooltip='Dijkstra').add_to(map_sg)

    # Plot A* route on the map
    if osm_a_star_path:
        a_star_path_coords = [(graph.nodes[node]['y'], graph.nodes[node]['x']) for node in osm_a_star_path]
        folium.PolyLine(locations=a_star_path_coords, color='red', weight=5, tooltip='A*').add_to(map_sg)

    # Save and display the map
    map_sg.save('mrt_route_map.html')
    map_sg

    return {
        'dijkstra': {
            'path': dijkstra_shortest_path,
            'distance': dijkstra_distance,
            'emissions': dijkstra_emissions,
            'stops': dijkstra_stops
        },
        'a_star': {
            'path': a_star_shortest_path,
            'distance': a_star_distance,
            'emissions': a_star_emissions,
            'stops': a_star_stops
        }
    }

if __name__ == "__main__":
    # For testing purposes
    start_station = 'CHANGI AIRPORT MRT STATION'
    end_station = 'BUKIT PANJANG MRT STATION'
    generate_and_display_paths(start_station, end_station)
