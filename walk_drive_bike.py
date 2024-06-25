import osmnx as ox
import heapq
import folium
from collections import defaultdict
import math

#network_type for getting nodes
TRANSPORT_TYPE = 'walk' # replace with 'walk' 'bike' 'drive' to change transport type

# Retrieve the walking network for Singapore
G = ox.graph_from_place('Singapore', network_type=TRANSPORT_TYPE)

# Extract the nodes from the graph
nodes = list(G.nodes)
print(f"Number of nodes in the walking network: {len(nodes)}")

# Define start and end coordinates (latitude, longitude)
start_coords = (1.3782561,103.846097)  # Example: SIT@NYP
end_coords = (1.3690023,103.8432938)    # Example: Some other point in Singapore

# Get the nearest nodes to the start and end coordinates
start_node = ox.distance.nearest_nodes(G, X=start_coords[1], Y=start_coords[0])
end_node = ox.distance.nearest_nodes(G, X=end_coords[1], Y=end_coords[0])

# Custom Dijkstra's algorithm implementation for shortest path
def custom_dijkstra(graph, start, end):
    queue = [(0, start, [])]
    seen = set()
    while queue:
        (cost, node, path) = heapq.heappop(queue)
        if node in seen:
            continue
        path = path + [node]
        seen.add(node)
        if node == end:
            return cost, path
        for neighbor, data in graph[node].items():
            if neighbor not in seen:
                heapq.heappush(queue, (cost + data.get('length', 1), neighbor, path))
    return float("inf"), []

# Custom A* algorithm implementation for shortest path
def custom_a_star(graph, start, end):
    def heuristic(u, v):
        u_coords = (graph.nodes[u]['y'], graph.nodes[u]['x'])
        v_coords = (graph.nodes[v]['y'], graph.nodes[v]['x'])
        return math.sqrt((u_coords[0] - v_coords[0])**2 + (u_coords[1] - v_coords[1])**2)
    
    open_set = [(0, start)]
    came_from = {}
    g_score = defaultdict(lambda: float('inf'))
    f_score = defaultdict(lambda: float('inf'))
    g_score[start] = 0
    f_score[start] = heuristic(start, end)
    
    while open_set:
        current = heapq.heappop(open_set)[1]
        if current == end:
            total_path = []
            while current in came_from:
                total_path.insert(0, current)
                current = came_from[current]
            return g_score[end], [start] + total_path
        
        for neighbor, data in graph[current].items():
            tentative_g_score = g_score[current] + data.get('length', 1)
            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, end)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
    
    return float('inf'), []

# Custom Bellman-Ford algorithm implementation for shortest path
def custom_bellman_ford(graph, start, end):
    distance = {node: float('inf') for node in graph.nodes}
    distance[start] = 0
    predecessor = {node: None for node in graph.nodes}
    
    for _ in range(len(graph.nodes) - 1):
        for u in graph.nodes:
            for v, data in graph[u].items():
                if distance[u] + data.get('length', 1) < distance[v]:
                    distance[v] = distance[u] + data.get('length', 1)
                    predecessor[v] = u
    
    path = []
    current = end
    while current is not None:
        path.insert(0, current)
        current = predecessor[current]
    
    return distance[end], path

# Find shortest path using custom Dijkstra's algorithm
dijkstra_cost, dijkstra_path = custom_dijkstra(G, start_node, end_node)
print(f"Shortest path using Dijkstra's algorithm from node {start_node} to node {end_node} has cost {dijkstra_cost} and path {dijkstra_path}")

# Find shortest path using custom A* algorithm
a_star_cost, a_star_path = custom_a_star(G, start_node, end_node)
print(f"Shortest path using A* algorithm from node {start_node} to node {end_node} has cost {a_star_cost} and path {a_star_path}")

# Find shortest path using custom Bellman-Ford algorithm
bellman_ford_cost, bellman_ford_path = custom_bellman_ford(G, start_node, end_node)
print(f"Shortest path using Bellman-Ford algorithm from node {start_node} to node {end_node} has cost {bellman_ford_cost} and path {bellman_ford_path}")

# Create a Folium map centered on Singapore
singapore_latlng = [1.3521, 103.8198]  # Coordinates for Singapore
m = folium.Map(location=singapore_latlng, zoom_start=12)

# Extract the coordinates of the nodes in each path
dijkstra_path_coords = [(G.nodes[node]['y'], G.nodes[node]['x']) for node in dijkstra_path]
a_star_path_coords = [(G.nodes[node]['y'], G.nodes[node]['x']) for node in a_star_path]
bellman_ford_path_coords = [(G.nodes[node]['y'], G.nodes[node]['x']) for node in bellman_ford_path]

# Add the routes to the Folium map with different colors
folium.PolyLine(dijkstra_path_coords, color="blue", weight=2.5, opacity=1, tooltip='Dijkstra').add_to(m)
folium.PolyLine(a_star_path_coords, color="green", weight=2.5, opacity=1, tooltip='A*').add_to(m)
folium.PolyLine(bellman_ford_path_coords, color="red", weight=2.5, opacity=1, tooltip='Bellman-Ford').add_to(m)

# Mark the start and end points
folium.Marker(location=dijkstra_path_coords[0], popup='Start', icon=folium.Icon(color='green')).add_to(m)
folium.Marker(location=dijkstra_path_coords[-1], popup='End', icon=folium.Icon(color='red')).add_to(m)

# Save map to an HTML file
m.save("singapore_route.html")
m
