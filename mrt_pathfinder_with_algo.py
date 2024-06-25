import pandas as pd
import networkx as nx
import osmnx as ox
import folium
import heapq
import webbrowser
import os

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

# Dijkstra's Algorithm Function for MRT graph
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
            return path[::-1]

        if current_distance > distances[current_node]:
            continue

        for neighbor in graph.neighbors(current_node):
            weight = graph[current_node][neighbor].get('length', 1)
            distance = current_distance + weight

            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous_nodes[neighbor] = current_node
                heapq.heappush(queue, (distance, neighbor))

    return []

# A* Algorithm Function for MRT graph
def heuristic(node1, node2):
    (x1, y1) = G.nodes[node1]['pos']
    (x2, y2) = G.nodes[node2]['pos']
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

def a_star(graph, start, end):
    open_set = [(0, start)]
    g_costs = {node: float('infinity') for node in graph.nodes}
    f_costs = {node: float('infinity') for node in graph.nodes}
    previous_nodes = {node: None for node in graph.nodes}
    g_costs[start] = 0
    f_costs[start] = heuristic(start, end)

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
            weight = graph[current_node][neighbor].get('length', 1)
            tentative_g_cost = g_costs[current_node] + weight
            if tentative_g_cost < g_costs[neighbor]:
                previous_nodes[neighbor] = current_node
                g_costs[neighbor] = tentative_g_cost
                f_costs[neighbor] = tentative_g_cost + heuristic(neighbor, end)
                heapq.heappush(open_set, (f_costs[neighbor], neighbor))

    return []

# Create the OSM graph
def create_osm_graph():
    place_name = "Singapore"
    return ox.graph_from_place(place_name, network_type='all', truncate_by_edge=True, simplify=True)

# Find the nearest nodes in the OSM network for each MRT station
def find_nearest_nodes(graph):
    nearest_nodes = {}
    for idx, row in mrt_stations.iterrows():
        nearest_node = ox.distance.nearest_nodes(graph, X=row['Longitude'], Y=row['Latitude'])
        nearest_nodes[row['STN_NAME']] = nearest_node
    return nearest_nodes

# Dijkstra's Algorithm Function for OSM graph
def dijkstra_osm(graph, start, end):
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
            return path[::-1]

        if current_distance > distances[current_node]:
            continue

        for neighbor in graph.neighbors(current_node):
            weight = graph.edges[current_node, neighbor, 0].get('length', 1)
            distance = current_distance + weight

            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous_nodes[neighbor] = current_node
                heapq.heappush(queue, (distance, neighbor))

    return []

# A* Algorithm Function for OSM graph
def a_star_osm(graph, start, end):
    def heuristic(node1, node2):
        (x1, y1) = (graph.nodes[node1]['x'], graph.nodes[node1]['y'])
        (x2, y2) = (graph.nodes[node2]['x'], graph.nodes[node2]['y'])
        return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

    open_set = [(0, start)]
    g_costs = {node: float('infinity') for node in graph.nodes}
    f_costs = {node: float('infinity') for node in graph.nodes}
    previous_nodes = {node: None for node in graph.nodes}
    g_costs[start] = 0
    f_costs[start] = heuristic(start, end)

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
            weight = graph.edges[current_node, neighbor, 0].get('length', 1)
            tentative_g_cost = g_costs[current_node] + weight
            if tentative_g_cost < g_costs[neighbor]:
                previous_nodes[neighbor] = current_node
                g_costs[neighbor] = tentative_g_cost
                f_costs[neighbor] = tentative_g_cost + heuristic(neighbor, end)
                heapq.heappush(open_set, (f_costs[neighbor], neighbor))

    return []

# Convert MRT shortest path to OSM shortest path for plotting
def convert_to_osm_path(shortest_path, nearest_nodes, graph, algorithm='dijkstra'):
    osm_path = []
    for i in range(len(shortest_path) - 1):
        start_node = nearest_nodes[shortest_path[i]]
        end_node = nearest_nodes[shortest_path[i + 1]]
        if algorithm == 'dijkstra':
            path = dijkstra_osm(graph, start_node, end_node)
        elif algorithm == 'a_star':
            path = a_star_osm(graph, start_node, end_node)
        else:
            raise ValueError("Invalid algorithm specified. Use 'dijkstra' or 'a_star'.")
        if not path:
            print(f"No path between {shortest_path[i]} and {shortest_path[i + 1]}")
            osm_path = []
            break
        osm_path.extend(path)
    return osm_path

def generate_and_display_paths(start_station, end_station):
    # Find paths using Dijkstra and A* algorithms
    dijkstra_shortest_path = dijkstra(G, start_station, end_station)
    a_star_shortest_path = a_star(G, start_station, end_station)

    # Create OSM graph and find nearest nodes
    osm_graph = create_osm_graph()
    nearest_nodes = find_nearest_nodes(osm_graph)

    # Convert paths to OSM paths using Dijkstra and A* algorithms
    osm_dijkstra_path = convert_to_osm_path(dijkstra_shortest_path, nearest_nodes, osm_graph, algorithm='dijkstra')
    osm_a_star_path = convert_to_osm_path(a_star_shortest_path, nearest_nodes, osm_graph, algorithm='a_star')

    # Create a folium map centered around Singapore
    map_sg = folium.Map(location=[1.3521, 103.8198], zoom_start=12)

    # Add markers to MRT stations
    for idx, row in mrt_stations.iterrows():
        folium.Marker(location=[row['Latitude'], row['Longitude']], popup=row['STN_NAME']).add_to(map_sg)

    # Plot Dijkstra route on the map
    if osm_dijkstra_path:
        dijkstra_path_coords = [(osm_graph.nodes[node]['y'], osm_graph.nodes[node]['x']) for node in osm_dijkstra_path]
        folium.PolyLine(locations=dijkstra_path_coords, color='blue', weight=5, tooltip='Dijkstra').add_to(map_sg)

    # Plot A* route on the map
    if osm_a_star_path:
        a_star_path_coords = [(osm_graph.nodes[node]['y'], osm_graph.nodes[node]['x']) for node in osm_a_star_path]
        folium.PolyLine(locations=a_star_path_coords, color='red', weight=5, tooltip='A*').add_to(map_sg)

    # Save map
    map_sg.save('./mrt_route_map.html')

    # Open map in default browser
    webbrowser.open('file://' + os.path.realpath('./mrt_route_map.html'))

    return dijkstra_shortest_path, a_star_shortest_path

if __name__ == "__main__":
    # For testing for now
    start_station = 'CHANGI AIRPORT MRT STATION'
    end_station = 'BUKIT PANJANG MRT STATION'
    generate_and_display_paths(start_station, end_station)
