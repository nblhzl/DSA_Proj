import tkinter as tk
from tkinter import ttk, messagebox
import webbrowser
import subprocess
import pandas as pd
from tkintermapview import TkinterMapView

class RoutePlannerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Route Planner")

        self.mode_var = tk.StringVar(value='car')
        self.start_var = tk.StringVar()
        self.end_var = tk.StringVar()

        # Load MRT stations
        self.mrt_stations = self.load_mrt_stations()
        self.create_widgets()

    def load_mrt_stations(self):
        mrt_stations_file = './MRT_Stations.csv'
        mrt_stations_df = pd.read_csv(mrt_stations_file)
        return mrt_stations_df['STN_NAME'].tolist()

    def create_widgets(self):
        # Create a canvas to enable scrolling
        canvas = tk.Canvas(self.root)
        scrollbar = ttk.Scrollbar(self.root, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)

        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(
                scrollregion=canvas.bbox("all")
            )
        )

        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.grid(row=0, column=0, sticky="nsew")
        scrollbar.grid(row=0, column=1, sticky="ns")

        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_columnconfigure(0, weight=1)
        
        # Mode selection
        ttk.Label(scrollable_frame, text="Select Mode of Transportation:").grid(row=0, column=0, padx=10, pady=10, sticky='w')
        modes = ['Car', 'Bike', 'Walking', 'MRT', 'Multi-Transport']
        for idx, mode in enumerate(modes):
            ttk.Radiobutton(scrollable_frame, text=mode, variable=self.mode_var, value=mode.lower(), command=self.update_inputs).grid(row=0, column=idx+1, padx=5, pady=10, sticky='w')

        # Start and end points input
        ttk.Label(scrollable_frame, text="Start Point:").grid(row=1, column=0, padx=10, pady=10, sticky='w')
        self.start_entry = ttk.Entry(scrollable_frame, textvariable=self.start_var)
        self.start_entry.grid(row=1, column=1, columnspan=4, padx=10, pady=10, sticky='ew')

        self.start_dropdown = ttk.Combobox(scrollable_frame, textvariable=self.start_var, values=self.mrt_stations)
        self.start_dropdown.grid(row=1, column=1, columnspan=4, padx=10, pady=10, sticky='ew')
        self.start_dropdown.grid_remove()

        ttk.Label(scrollable_frame, text="End Point:").grid(row=2, column=0, padx=10, pady=10, sticky='w')
        self.end_entry = ttk.Entry(scrollable_frame, textvariable=self.end_var)
        self.end_entry.grid(row=2, column=1, columnspan=4, padx=10, pady=10, sticky='ew')

        self.end_dropdown = ttk.Combobox(scrollable_frame, textvariable=self.end_var, values=self.mrt_stations)
        self.end_dropdown.grid(row=2, column=1, columnspan=4, padx=10, pady=10, sticky='ew')
        self.end_dropdown.grid_remove()

        # Map view for setting start and end points
        self.map_widget = TkinterMapView(scrollable_frame, width=800, height=600, corner_radius=0)
        self.map_widget.grid(row=3, column=0, columnspan=5, padx=10, pady=10, sticky='nsew')
        self.map_widget.set_position(1.3521, 103.8198)  # Center the map on Singapore
        self.map_widget.set_zoom(12)

        self.map_widget.add_left_click_map_command(self.set_marker)

        self.start_marker = None
        self.end_marker = None

        # Route display textbox
        self.route_text = tk.Text(scrollable_frame, height=20, width=100)
        self.route_text.grid(row=4, column=0, columnspan=5, padx=10, pady=10, sticky='ew')

        # Run button
        ttk.Button(scrollable_frame, text="Generate Route", command=self.run_script).grid(row=5, column=0, columnspan=5, pady=20)

        # Configure column and row weights
        scrollable_frame.grid_columnconfigure(0, weight=1)
        scrollable_frame.grid_columnconfigure(1, weight=1)
        scrollable_frame.grid_columnconfigure(2, weight=1)
        scrollable_frame.grid_columnconfigure(3, weight=1)
        scrollable_frame.grid_columnconfigure(4, weight=1)
        scrollable_frame.grid_rowconfigure(3, weight=1)
        scrollable_frame.grid_rowconfigure(4, weight=1)

    def set_marker(self, coords):
        if not self.start_marker:
            self.start_marker = self.map_widget.set_marker(coords[0], coords[1], text="Start")
            self.start_var.set(f"{coords[0]}, {coords[1]}")
        elif not self.end_marker:
            self.end_marker = self.map_widget.set_marker(coords[0], coords[1], text="End")
            self.end_var.set(f"{coords[0]}, {coords[1]}")
        else:
            messagebox.showinfo("Markers Set", "Both start and end markers have been set. Please generate the route.")

    def update_inputs(self):
        mode = self.mode_var.get()
        if mode == 'mrt':
            self.start_entry.grid_remove()
            self.end_entry.grid_remove()
            self.start_dropdown.grid()
            self.end_dropdown.grid()
            self.map_widget.grid_remove()
            self.start_var.set('')
            self.end_var.set('')
            self.clear_markers()
        else:
            self.start_entry.grid_remove()
            self.end_entry.grid_remove()
            self.start_dropdown.grid_remove()
            self.end_dropdown.grid_remove()
            self.map_widget.grid()
            self.clear_markers()
            self.start_var.set('')
            self.end_var.set('')

    def clear_markers(self):
        if self.start_marker or self.end_marker:
            self.map_widget.delete_all_marker()
            self.start_marker = None
            self.end_marker = None

    def run_script(self):
        mode = self.mode_var.get()
        start = self.start_var.get()
        end = self.end_var.get()

        if mode == 'mrt':
            if not start or not end:
                messagebox.showerror("Input Error", "Please select valid MRT stations.")
                return
            script = 'mrt_pathfinder_with_alt.py'
            args = [script, start, end]
        else:
            if not start or not end:
                messagebox.showerror("Input Error", "Please set valid start and end points on the map.")
                return
            script = 'drive_bike_walk_pathfinder.py' if mode in ['car', 'bike', 'walking'] else 'multi_transport_pathfinder.py'
            start_coords = start.split(',')
            end_coords = end.split(',')
            args = [script, start_coords[0].strip(), start_coords[1].strip(), end_coords[0].strip(), end_coords[1].strip()]

        try:
            result = subprocess.run(['python'] + args, check=True, capture_output=True, text=True)
            output = result.stdout.strip().split('\n')
            if not output:
                messagebox.showerror("Execution Error", "No output file generated.")
                return

            # Display routes in the textbox
            self.route_text.delete(1.0, tk.END)
            for line in output:
                self.route_text.insert(tk.END, line + "\n")
                if line.endswith('.html'):
                    webbrowser.open(line)
        except subprocess.CalledProcessError as e:
            messagebox.showerror("Execution Error", f"An error occurred while running the script:\n{e.stderr}")

if __name__ == "__main__":
    root = tk.Tk()
    app = RoutePlannerApp(root)
    root.mainloop()
