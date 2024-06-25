import tkinter as tk
from tkinter import ttk, messagebox
from mrt_pathfinder_with_algo import mrt_stations, generate_and_display_paths

class MRTPathFinderApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("DSAG Proj")
        self.geometry("600x400")

        self.create_widgets()

    def create_widgets(self):
        # Labels and dropdowns for selecting start and end stations
        self.start_label = tk.Label(self, text="Starting Station:")
        self.start_label.pack(pady=5)
        self.start_station = ttk.Combobox(self, values=list(mrt_stations['STN_NAME']))
        self.start_station.pack(pady=5)

        self.end_label = tk.Label(self, text="End Station:")
        self.end_label.pack(pady=5)
        self.end_station = ttk.Combobox(self, values=list(mrt_stations['STN_NAME']))
        self.end_station.pack(pady=5)

        # Button to find paths and display the map
        self.find_button = tk.Button(self, text="Find Path", command=self.find_path)
        self.find_button.pack(pady=20)

        # Text widget to display the shortest path
        self.result_text = tk.Text(self, height=10, width=50)
        self.result_text.pack(pady=10)

    def find_path(self):
        start = self.start_station.get()
        end = self.end_station.get()
        if not start or not end:
            messagebox.showerror("Error", "Select start and end stations")
            return
        dijkstra_path, a_star_path = generate_and_display_paths(start, end)
        self.result_text.delete(1.0, tk.END)
        self.result_text.insert(tk.END, f"Dijkstra shortest path: {dijkstra_path}\n")
        self.result_text.insert(tk.END, f"A* shortest path: {a_star_path}\n")
        messagebox.showinfo("Info", f"{start} to {end}")

if __name__ == "__main__":
    print("Start")
    app = MRTPathFinderApp()
    app.mainloop()
