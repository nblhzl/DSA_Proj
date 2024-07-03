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

# Emission factors in grams of CO₂ per km
emission_factors = {
    'mrt': 28,  # grams of CO₂ per km for MRT
    'drive': 170  # grams of CO₂ per km for petrol car
}

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
            return path[::-1], distances[end]

        if current_distance > distances[current_node]:
            continue

        for neighbor in graph.neighbors(current_node):
            if is_osm:
                weight = graph.edges[current_node, neighbor, 0].get('length', 1)
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
        (x1, y1) = (graph.nodes[node1]['x'], graph.nodes[node1]['y'])
        (x2, y2) = (graph.nodes[node2]['x'], graph.nodes[node2]['y'])
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

def calculate_emissions(distance_km, mode):
    return (distance_km * emission_factors[mode]) / 1000  # return emissions in kg

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

def generate_and_display_paths_multi_transport(start, end):
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

    # Find the coordinates of the start and end stations
    start_coords = (mrt_stations.loc[mrt_stations['STN_NAME'] == start_station, 'Latitude'].values[0],
                    mrt_stations.loc[mrt_stations['STN_NAME'] == start_station, 'Longitude'].values[0])
    end_coords = (mrt_stations.loc[mrt_stations['STN_NAME'] == end_station, 'Latitude'].values[0],
                  mrt_stations.loc[mrt_stations['STN_NAME'] == end_station, 'Longitude'].values[0])

    # Download Singapore street network using osmnx
    place_name = "Singapore"
    graph = ox.graph_from_place(place_name, network_type='all', truncate_by_edge=True, simplify=True)

    # Find the nearest nodes in the OSM network for each MRT station
    nearest_nodes = {}
    for idx, row in mrt_stations.iterrows():
        nearest_node = ox.distance.nearest_nodes(graph, X=row['Longitude'], Y=row['Latitude'])
        nearest_nodes[row['STN_NAME']] = nearest_node

    # Find the nearest nodes in the OSM network for start and end coordinates
    start_node = ox.distance.nearest_nodes(graph, X=start_coords[1], Y=start_coords[0])
    end_node = ox.distance.nearest_nodes(graph, X=end_coords[1], Y=end_coords[0])

    # Find the nearest MRT station to the start and end coordinates
    nearest_start_station = find_nearest_station(start_coords)
    nearest_end_station = find_nearest_station(end_coords)

    # Use the generalized Dijkstra function to find the shortest path in MRT network
    dijkstra_shortest_path, dijkstra_mrt_distance = dijkstra(G, nearest_start_station, nearest_end_station)
    print("Dijkstra MRT shortest path:", dijkstra_shortest_path)

    # Use the generalized A* function to find the shortest path in MRT network
    a_star_shortest_path, a_star_mrt_distance = a_star(G, nearest_start_station, nearest_end_station)
    print("A* MRT shortest path:", a_star_shortest_path)

    # Calculate MRT emissions
    dijkstra_mrt_emissions = calculate_emissions(dijkstra_mrt_distance / 1000, 'mrt')
    a_star_mrt_emissions = calculate_emissions(a_star_mrt_distance / 1000, 'mrt')

    # Convert MRT shortest path to OSM shortest path for plotting
    def convert_to_osm_path(shortest_path, algorithm):
        osm_path = []
        total_distance = 0
        for i in range(len(shortest_path) - 1):
            start_node = nearest_nodes[shortest_path[i]]
            end_node = nearest_nodes[shortest_path[i + 1]]
            if algorithm == 'dijkstra':
                path, distance = dijkstra(graph, start_node, end_node, is_osm=True)
            elif algorithm == 'a_star':
                path, distance = a_star(graph, start_node, end_node, coord_attr='x', is_osm=True)
            else:
                raise ValueError("Invalid algorithm specified. Use 'dijkstra' or 'a_star'.")
            if not path:
                print(f"No path between {shortest_path[i]} and {shortest_path[i + 1]}")
                osm_path = []
                break
            osm_path.extend(path)
            total_distance += distance
        return osm_path, total_distance

    # Convert paths to OSM paths using Dijkstra and A* algorithms
    osm_dijkstra_path, osm_dijkstra_distance = convert_to_osm_path(dijkstra_shortest_path, 'dijkstra')
    osm_a_star_path, osm_a_star_distance = convert_to_osm_path(a_star_shortest_path, 'a_star')

    # Find the walking/biking paths from the start coordinate to the nearest MRT station and from the nearest MRT station to the end coordinate
    start_to_mrt_dijkstra_path, start_to_mrt_dijkstra_distance = dijkstra(graph, start_node, nearest_nodes[nearest_start_station], is_osm=True)
    end_to_mrt_dijkstra_path, end_to_mrt_dijkstra_distance = dijkstra(graph, nearest_nodes[nearest_end_station], end_node, is_osm=True)

    start_to_mrt_a_star_path, start_to_mrt_a_star_distance = a_star(graph, start_node, nearest_nodes[nearest_start_station], coord_attr='x', is_osm=True)
    end_to_mrt_a_star_path, end_to_mrt_a_star_distance = a_star(graph, nearest_nodes[nearest_end_station], end_node, coord_attr='x', is_osm=True)

    # Create folium maps centered around Singapore for Dijkstra and A*
    map_dijkstra = folium.Map(location=[1.3521, 103.8198], zoom_start=12)
    map_a_star = folium.Map(location=[1.3521, 103.8198], zoom_start=12)

    # Function to plot paths
    def plot_paths(map_obj, start_to_mrt_path, mrt_path, mrt_to_end_path, start_to_mrt_distance, mrt_distance, mrt_to_end_distance, mrt_color, algorithm_name):
        # Add markers to MRT stations
        for idx, row in mrt_stations.iterrows():
            folium.Marker(location=[row['Latitude'], row['Longitude']], popup=row['STN_NAME']).add_to(map_obj)

        # Plot walking/biking path from start coordinate to nearest MRT station
        if start_to_mrt_path:
            start_to_mrt_coords = [(graph.nodes[node]['y'], graph.nodes[node]['x']) for node in start_to_mrt_path]
            print("Distance from Start coords to MRT station:", start_to_mrt_distance)
            if start_to_mrt_distance <= 1: # plot a walk route if distance is less than 1km
                folium.PolyLine(locations=start_to_mrt_coords, color='green', weight=5, tooltip=f'Walking Start to MRT ({algorithm_name})').add_to(map_obj)
            else:
                folium.PolyLine(locations=start_to_mrt_coords, color='purple', weight=5, tooltip=f'Biking Start to MRT ({algorithm_name})').add_to(map_obj)

        # Plot walking/biking path from nearest MRT station to end coordinate
        if mrt_to_end_path:
            mrt_to_end_coords = [(graph.nodes[node]['y'], graph.nodes[node]['x']) for node in mrt_to_end_path]
            print("Distance from MRT station to end coords:", mrt_to_end_distance)
            if mrt_to_end_distance <= 1: # plot a walk route if distance is less than 1km
                folium.PolyLine(locations=mrt_to_end_coords, color='green', weight=5, tooltip=f'Walking MRT to End ({algorithm_name})').add_to(map_obj)
            else:
                folium.PolyLine(locations=mrt_to_end_coords, color='purple', weight=5, tooltip=f'Biking MRT to End ({algorithm_name})').add_to(map_obj)

        # Plot MRT route on the map
        if mrt_path:
            mrt_path_coords = [(graph.nodes[node]['y'], graph.nodes[node]['x']) for node in mrt_path]
            folium.PolyLine(locations=mrt_path_coords, color=mrt_color, weight=5, tooltip=f'{algorithm_name} MRT').add_to(map_obj)

    # Plot paths on the Dijkstra map
    plot_paths(map_dijkstra, start_to_mrt_dijkstra_path, osm_dijkstra_path, end_to_mrt_dijkstra_path, start_to_mrt_dijkstra_distance, dijkstra_mrt_distance, end_to_mrt_dijkstra_distance, 'blue', 'Dijkstra')

    # Plot paths on the A* map
    plot_paths(map_a_star, start_to_mrt_a_star_path, osm_a_star_path, end_to_mrt_a_star_path, start_to_mrt_a_star_distance, a_star_mrt_distance, end_to_mrt_a_star_distance, 'red', 'A*')

    # Save and display the maps
    map_dijkstra.save('multi_transport_dijkstra.html')
    map_a_star.save('multi_transport_a_star.html')

    results = {
        "dijkstra": {
            "path": dijkstra_shortest_path,
            "distance": dijkstra_mrt_distance,
            "emissions": dijkstra_mrt_emissions
        },
        "a_star": {
            "path": a_star_shortest_path,
            "distance": a_star_mrt_distance,
            "emissions": a_star_mrt_emissions
        }
    }

    return results

if __name__ == "__main__":
    # For testing purposes
    start_station = '1.390505,103.986793'  # Coordinates instead of station name
    end_station = '1.379192,103.759387'    # Coordinates instead of station name
    results = generate_and_display_paths_multi_transport(start_station, end_station)
    print("Results:", results)
