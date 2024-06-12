import pandas as pd
import networkx as nx
import osmnx as ox
import folium

# Load the MRT stations and edges CSV files
stations_file_path = 'C:/Users/joelc/Downloads/DSAG_Project/MRT_Stations.csv'
edges_file_path = 'C:/Users/joelc/Downloads/DSAG_Project/MRT_Stations_Edges.csv'

mrt_stations = pd.read_csv(stations_file_path)
edges = pd.read_csv(edges_file_path)

# Create a new NetworkX graph
G = nx.Graph()

# Add nodes with positions for each MRT station
for idx, row in mrt_stations.iterrows():
    G.add_node(row['STN_NAME'], pos=(row['Longitude'], row['Latitude']))

# Add edges between the MRT stations based on the edges DataFrame
for idx, row in edges.iterrows():
    G.add_edge(row['Station'], row['Connected Station'])

# Input start and end mrt station for now
start_station = 'CHANGI AIRPORT MRT STATION'
end_station = 'WOODLANDS MRT STATION'

# Find the shortest path between the start and end stations
try:
    shortest_path = nx.shortest_path(G, source=start_station, target=end_station)
    print("Shortest path:", shortest_path)
except nx.NetworkXNoPath:
    print(f"No path between {start_station} and {end_station}")
    shortest_path = []

# Download the street network for Singapore using osmnx
place_name = "Singapore"
graph = ox.graph_from_place(place_name, network_type='all')

# Find the nearest nodes in the OSM network for each MRT station
nearest_nodes = {}
for idx, row in mrt_stations.iterrows():
    nearest_node = ox.distance.nearest_nodes(graph, X=row['Longitude'], Y=row['Latitude'])
    nearest_nodes[row['STN_NAME']] = nearest_node

# Find the shortest path in the OSM network using the nearest nodes
if shortest_path:
    osm_path = []
    for i in range(len(shortest_path) - 1):
        start_node = nearest_nodes[shortest_path[i]]
        end_node = nearest_nodes[shortest_path[i + 1]]
        try:
            path = nx.shortest_path(graph, source=start_node, target=end_node, weight='length')
            osm_path.extend(path)
        except nx.NetworkXNoPath:
            print(f"No path between {shortest_path[i]} and {shortest_path[i + 1]}")
            osm_path = []
            break

# Create a folium map centered around Singapore
map_sg = folium.Map(location=[1.3521, 103.8198], zoom_start=12)

# Add MRT stations as markers
for idx, row in mrt_stations.iterrows():
    folium.Marker(location=[row['Latitude'], row['Longitude']], popup=row['STN_NAME']).add_to(map_sg)

# Plot the realistic path on the map
if osm_path:
    path_coords = [(graph.nodes[node]['y'], graph.nodes[node]['x']) for node in osm_path]
    folium.PolyLine(locations=path_coords, color='blue', weight=5).add_to(map_sg)

# Save and display the map
map_sg.save('shortest_path_map.html')
map_sg
