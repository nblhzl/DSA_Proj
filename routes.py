import osmnx as ox
import networkx as nx
import folium
import heapq
import time

# Custom Dijkstra Algorithm
def custom_dijkstra(graph, start_node, end_node):
    queue = [(0, start_node)]
    distances = {node: float('inf') for node in graph.nodes}
    previous_nodes = {node: None for node in graph.nodes}
    distances[start_node] = 0

    while queue:
        current_distance, current_node = heapq.heappop(queue)

        if current_node == end_node:
            break

        for neighbor, attributes in graph[current_node].items():
            weight = attributes.get('length', 1)
            distance = current_distance + weight

            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous_nodes[neighbor] = current_node
                heapq.heappush(queue, (distance, neighbor))

    path = []
    current_node = end_node
    while current_node is not None:
        path.append(current_node)
        current_node = previous_nodes[current_node]
    path.reverse()

    return path

# Custom A* Algorithm
def custom_astar(graph, start_node, end_node):
    def heuristic(u, v):
        return ox.distance.euclidean(graph.nodes[u]['y'], graph.nodes[u]['x'],
                                     graph.nodes[v]['y'], graph.nodes[v]['x'])

    open_set = [(0, start_node)]
    g_scores = {node: float('inf') for node in graph.nodes}
    f_scores = {node: float('inf') for node in graph.nodes}
    previous_nodes = {node: None for node in graph.nodes}
    g_scores[start_node] = 0
    f_scores[start_node] = heuristic(start_node, end_node)

    while open_set:
        _, current_node = heapq.heappop(open_set)

        if current_node == end_node:
            break

        for neighbor, attributes in graph[current_node].items():
            weight = attributes.get('length', 1)
            tentative_g_score = g_scores[current_node] + weight

            if tentative_g_score < g_scores[neighbor]:
                previous_nodes[neighbor] = current_node
                g_scores[neighbor] = tentative_g_score
                f_scores[neighbor] = g_scores[neighbor] + heuristic(neighbor, end_node)
                heapq.heappush(open_set, (f_scores[neighbor], neighbor))

    path = []
    current_node = end_node
    while current_node is not None:
        path.append(current_node)
        current_node = previous_nodes[current_node]
    path.reverse()

    return path

# Function to find shortest path using custom algorithms
def find_shortest_path_custom(start_location, end_location, mode='drive', algorithm='dijkstra'):
    start_coords = ox.geocode(start_location + ', Singapore')
    end_coords = ox.geocode(end_location + ', Singapore')
    
    network_type = mode
    graph = ox.graph_from_place('Singapore', network_type=network_type)
    
    start_node = ox.distance.nearest_nodes(graph, X=start_coords[1], Y=start_coords[0])
    end_node = ox.distance.nearest_nodes(graph, X=end_coords[1], Y=end_coords[0])
    
    start_time = time.time()
    if algorithm == 'dijkstra':
        path = custom_dijkstra(graph, start_node, end_node)
    elif algorithm == 'astar':
        path = custom_astar(graph, start_node, end_node)
    end_time = time.time()
    
    computation_time = end_time - start_time
    
    return path, graph, computation_time

# Main script
start_location = 'Redhill, Singapore'
end_location = 'Choa Chu Kang, Singapore'
algorithm = 'astar'  # 'dijkstra' or 'astar'

# Colors for different modes
colors = {
    'drive': 'blue',
    'bike': 'green',
    'walk': 'red'
}

# Initialize map
map_sg = folium.Map(location=[1.3521, 103.8198], zoom_start=12)

# Geocode start and end locations to get coordinates for markers
start_coords = ox.geocode(start_location)
end_coords = ox.geocode(end_location)

# Add markers for start and end points on the map
folium.Marker(location=[start_coords[0], start_coords[1]], popup='Start Point').add_to(map_sg)
folium.Marker(location=[end_coords[0], end_coords[1]], popup='End Point').add_to(map_sg)

# Calculate and plot paths for different modes
for mode in ['drive', 'bike', 'walk']:
    osm_path, graph, comp_time = find_shortest_path_custom(start_location, end_location, mode=mode, algorithm=algorithm)
    if osm_path:
        path_coords = [(graph.nodes[node]['y'], graph.nodes[node]['x']) for node in osm_path]
        folium.PolyLine(locations=path_coords, color=colors[mode], weight=5, opacity=0.6, popup=mode).add_to(map_sg)

# Save and display the map
map_sg.save(f'shortest_path_custom_all_modes_{algorithm}.html')

print(f"Computation times:")
for mode in ['drive', 'bike', 'walk']:
    osm_path, graph, comp_time = find_shortest_path_custom(start_location, end_location, mode=mode, algorithm=algorithm)
    print(f"{mode.capitalize()} ({algorithm}): {comp_time} seconds")

map_sg
