import osmnx as ox
import networkx as nx
import matplotlib.pyplot as plt
import tkinter as tk
from tkinter import messagebox

def calculate_route():
    try:
        origin_point = (float(entry_origin_lat.get()), float(entry_origin_lon.get()))
        destination_point = (float(entry_dest_lat.get()), float(entry_dest_lon.get()))
        mode = mode_var.get()

        avg_speed = {"drive": 50, "bike": 15, "walk": 5}
        
        # Download the street network data for Singapore
        G = ox.graph_from_place("Singapore", network_type='bike') # drive, bike, walk, all (need to take into consideration public transport. need another api?)
        
        # Get the nearest network nodes to the origin and destination points
        orig_node = ox.distance.nearest_nodes(G, X=origin_point[1], Y=origin_point[0]) # Example: Changi Airport = (1.3644, 103.9915)
        dest_node = ox.distance.nearest_nodes(G, X=destination_point[1], Y=destination_point[0]) # Example: Marina Bay Sands = (1.2834, 103.8607)
        
        # Calculate shortest path
        route = nx.shortest_path(G, orig_node, dest_node, weight='length')

        # Calculate route distance
        route_length = sum(ox.utils_graph.get_route_edge_attributes(G, route, 'length'))
        route_length_km = route_length / 1000

        # Calculate travel time
        travel_duration = route_length_km / avg_speed[mode]
        travel_duration_minutes = travel_duration * 60
        
        # Plot route
        fig, ax = ox.plot_graph_route(G, route, route_linewidth=6, node_size=0, bgcolor='k', figsize=(10, 10))
        plt.show()

        # Display distance and travel time
        messagebox.showinfo("Route Info", f"Distance: {route_length_km:.2f} km\nEstimated Travel Time: {travel_duration_minutes:.2f} minutes")
    
    except Exception as e:
        messagebox.showerror("Error", str(e))

# Create main window
root = tk.Tk()
root.title("Route Finder")

# Origin coordinates
tk.Label(root, text="Origin Latitude:").grid(row=0, column=0)
entry_origin_lat = tk.Entry(root)
entry_origin_lat.grid(row=0, column=1)

tk.Label(root, text="Origin Longitude:").grid(row=1, column=0)
entry_origin_lon = tk.Entry(root)
entry_origin_lon.grid(row=1, column=1)

# Destination coordinates
tk.Label(root, text="Destination Latitude:").grid(row=2, column=0)
entry_dest_lat = tk.Entry(root)
entry_dest_lat.grid(row=2, column=1)

tk.Label(root, text="Destination Longitude:").grid(row=3, column=0)
entry_dest_lon = tk.Entry(root)
entry_dest_lon.grid(row=3, column=1)

# Mode of transport
mode_var = tk.StringVar(value='drive')
tk.Label(root, text="Mode of Transport:").grid(row=4, column=2)
tk.Radiobutton(root, text="Drive", variable=mode_var, value='drive').grid(row=4, column=3)
tk.Radiobutton(root, text="Bike", variable=mode_var, value='bike').grid(row=5, column=3)
tk.Radiobutton(root, text="Walk", variable=mode_var, value='walk').grid(row=6, column=3)

# Calculate button
btn_calculate = tk.Button(root, text="Calculate Route", command=calculate_route)
btn_calculate.grid(row=4, column=0, columnspan=2)

root.mainloop()