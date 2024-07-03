import tkinter as tk
from tkinter import ttk, messagebox
import webbrowser
import os
from mrt_pathfinder_with_algo import mrt_stations as mrt_stations_algo, generate_and_display_paths as generate_algo_paths
from multi_transport_pathfinder import generate_and_display_paths_multi_transport as generate_multi_paths
from drive_bike_walk_pathfinder import generate_and_display_paths_drive_bike_walk as generate_drive_bike_walk_paths

class MRTPathFinderApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("DSAG Proj")
        self.geometry("600x500")

        self.create_widgets()

    def create_widgets(self):
        # for selecting transport mode
        self.mode_label = tk.Label(self, text="Transport Mode:")
        self.mode_label.pack(pady=5)
        self.transport_mode = ttk.Combobox(self, values=["MRT", "Multi-transport", "Drive", "Bike", "Walk"])
        self.transport_mode.pack(pady=5)
        self.transport_mode.bind("<<ComboboxSelected>>", self.update_input_fields)

        # for selecting start and end points
        self.start_label = tk.Label(self, text="Starting Point:")
        self.start_label.pack(pady=5)
        self.start_input = ttk.Combobox(self, values=list(mrt_stations_algo['STN_NAME']))
        self.start_input.pack(pady=5)

        self.end_label = tk.Label(self, text="End Point:")
        self.end_label.pack(pady=5)
        self.end_input = ttk.Combobox(self, values=list(mrt_stations_algo['STN_NAME']))
        self.end_input.pack(pady=5)

        # to find paths and display map
        self.find_button = tk.Button(self, text="Find Path", command=self.find_path)
        self.find_button.pack(pady=20)

        self.result_text = tk.Text(self, height=10, width=50)
        self.result_text.pack(pady=10)

    def update_input_fields(self, event):
        mode = self.transport_mode.get()
        if mode == "MRT" or mode == "Multi-transport":
            self.start_input.config(values=list(mrt_stations_algo['STN_NAME']))
            self.end_input.config(values=list(mrt_stations_algo['STN_NAME']))
        else:
            self.start_input.config(values=[], state="normal")
            self.end_input.config(values=[], state="normal")

    def find_path(self):
        start = self.start_input.get()
        end = self.end_input.get()
        mode = self.transport_mode.get()
        if not start or not end:
            messagebox.showerror("Error", "Select start and end points")
            return
        if not mode:
            messagebox.showerror("Error", "Select a transport mode")
            return

        self.result_text.delete(1.0, tk.END)

        if mode == "MRT":
            self.result_text.insert(tk.END, "Generating paths for MRT algorithm...\n")
            paths_info = generate_algo_paths(start, end)
            self.display_paths_info(paths_info, "MRT")
            self.open_map("mrt_route_map.html")

        elif mode == "Multi-transport":
            self.result_text.insert(tk.END, "Generating paths for multi-transport...\n")
            paths_info = generate_multi_paths(start, end)
            self.display_paths_info(paths_info, "Multi-transport")
            self.open_map("multi_transport_dijkstra.html")

        else:
            try:
                start_coords = tuple(map(float, start.split(',')))
                end_coords = tuple(map(float, end.split(',')))
                self.result_text.insert(tk.END, f"Generating paths for {mode.lower()}...\n")
                paths_info = generate_drive_bike_walk_paths(start_coords, end_coords)
                self.result_text.insert(tk.END, f"Paths generation complete for {mode.lower()}.\n")
                self.display_paths_info(paths_info[mode.lower()], mode)
                self.open_map(paths_info[mode.lower()]['file'])
            except ValueError:
                messagebox.showerror("Error", "Please enter valid coordinates in the format 'lat,lon' for start and end points")
                return

        messagebox.showinfo("Info", f"{start} to {end} by {mode}")

    def display_paths_info(self, paths_info, mode):
        for algorithm, info in paths_info.items():
            if algorithm == 'file':
                continue
            self.result_text.insert(tk.END, f"{algorithm.capitalize()} shortest path ({mode}): {info['path']}\n")
            self.result_text.insert(tk.END, f"Distance: {info['distance']:.2f} km\n")
            self.result_text.insert(tk.END, f"Emissions: {info['emissions']:.2f} kg CO2\n")
            if mode == "MRT":
                self.result_text.insert(tk.END, f"Number of stops: {info.get('stops', 'N/A')}\n")

    def open_map(self, filename):
        file_path = os.path.abspath(filename)
        webbrowser.open(f'file:///{file_path}')

if __name__ == "__main__":
    app = MRTPathFinderApp()
    app.mainloop()
