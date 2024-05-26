import osmnx as ox
import networkx as nx
import matplotlib.pyplot as plt
import tkinter as tk
from tkinter import messagebox

def calculate_route():
    try:
        origin_point = (float(entry_origin_lat.get()), float(entry_origin_lon.get()))
        destination_point = (float(entry_dest_lat.get()), float(entry_dest_lon.get()))
        
        # Download the street network data for Singapore
        G = ox.graph_from_place("Singapore", network_type='drive')
        
        # Get the nearest network nodes to the origin and destination points
        orig_node = ox.distance.nearest_nodes(G, X=origin_point[1], Y=origin_point[0])
        dest_node = ox.distance.nearest_nodes(G, X=destination_point[1], Y=destination_point[0])
        
        # Calculate shortest path
        route = nx.shortest_path(G, orig_node, dest_node, weight='length')
        
        # Plot route
        fig, ax = ox.plot_graph_route(G, route, route_linewidth=6, node_size=0, bgcolor='k', figsize=(10, 10))
        plt.show()
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

# Calculate button
btn_calculate = tk.Button(root, text="Calculate Route", command=calculate_route)
btn_calculate.grid(row=4, column=0, columnspan=2)

root.mainloop()
