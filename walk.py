import osmnx as ox
import networkx as nx
import folium

# Retrieve the walking network for Singapore
G = ox.graph_from_place('Singapore', network_type='walk')

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
    import heapq

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

# Find shortest path using custom Dijkstra's algorithm
cost, path = custom_dijkstra(G, start_node, end_node)

# Display the result
print(f"Shortest path from node {start_node} to node {end_node} has cost {cost} and path {path}")

# Create a Folium map centered on Singapore
singapore_latlng = [1.3521, 103.8198]  # Coordinates for Singapore
m = folium.Map(location=singapore_latlng, zoom_start=12)

# Extract the coordinates of the nodes in the path
path_coords = [(G.nodes[node]['y'], G.nodes[node]['x']) for node in path]

# Add the route to the Folium map
folium.PolyLine(path_coords, color="blue", weight=2.5, opacity=1).add_to(m)

# Mark the start and end points
folium.Marker(location=path_coords[0], popup='Start', icon=folium.Icon(color='green')).add_to(m)
folium.Marker(location=path_coords[-1], popup='End', icon=folium.Icon(color='red')).add_to(m)

# Save map to an HTML file
m.save("singapore_route.html")
m