import os
import sys
import json

from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QRadioButton, QLineEdit, QComboBox, QPushButton, QTextEdit, QMessageBox
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtCore import QUrl, pyqtSignal, QObject, pyqtSlot
from PyQt5.QtWebChannel import QWebChannel
import folium

import pandas as pd
import subprocess

class EmbedMap(QMainWindow):
    def __init__(self, map_path=None):
        super().__init__()
        self.map_path = map_path
        self.setWindowTitle("Map Viewer")
        self.setGeometry(100, 100, 800, 600)

        layout = QVBoxLayout()
        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

        self.map_file = ""
        self.map_label = QLabel("Map file: ")
        layout.addWidget(self.map_label)

        self.emissions_label = QLabel("Emissions: ")
        layout.addWidget(self.emissions_label)

        self.web_view = QWebEngineView()
        self.web_view.setUrl(QUrl.fromLocalFile(self.map_path))
        layout.addWidget(self.web_view)
    
    def load_map(self, map_path):
        if os.path.exists(map_path):
            self.map_path = os.path.abspath(map_path)
            self.web_view.setUrl(QUrl.fromLocalFile(self.map_path))
            self.map_file = os.path.basename(self.map_path)
            self.map_label.setText("Map file: " + self.map_file)

    def update_emissions(self, emissions):
        self.emissions_label.setText(f"Emissions: {emissions}")

class MapManager(QObject):
    clicked = pyqtSignal(float, float)
    marker_dragged = pyqtSignal(str, float, float)

    @pyqtSlot(str, str)
    def receive_data(self, message, json_data):
        data = json.loads(json_data)
        if message == "click":
            self.clicked.emit(data["lat"], data["lng"])
        elif message == "dragend":
            self.marker_dragged.emit(data["type"], data["lat"], data["lng"])

class MapView(QMainWindow):
    start_coordinates_changed = pyqtSignal(float, float)
    end_coordinates_changed = pyqtSignal(float, float)
    clear_button_pressed = pyqtSignal(str)

    def __init__(self):
        super().__init__()

        self.map = folium.Map(location=[1.3521, 103.8198], zoom_start=12)  # Singapore coordinates

        self.click_count = 0
        self.start_marker = None
        self.end_marker = None

        self.map_path = os.path.abspath("./map.html")

        self.save_map()

        self.setWindowTitle("Map Viewer")
        self.setGeometry(100, 100, 800, 600)

        layout = QVBoxLayout()
        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

        self.web_view = QWebEngineView()

        channel = QWebChannel(self.web_view)
        map_manager = MapManager(self)
        channel.registerObject("map_manager", map_manager)
        self.web_view.page().setWebChannel(channel)
        
        self.web_view.setUrl(QUrl.fromLocalFile(self.map_path))
        layout.addWidget(self.web_view)

        self.clear_button = QPushButton("Clear Markers")
        self.clear_button.clicked.connect(self.clear_markers)

        layout.addWidget(self.clear_button)

        map_manager.clicked.connect(self.handle_click)
        map_manager.marker_dragged.connect(self.handle_marker_dragged)

    def handle_click(self, lat, lng):
        if self.click_count > 1:
            return
        if self.click_count == 0:
            self.start_marker = folium.Marker(
                location=[lat, lng],
                draggable=True,
                icon=folium.Icon(color='blue'),
                popup=f"Start: ({lat}, {lng})"
            )
            self.start_marker.add_to(self.map)
            self.click_count += 1
            self.start_coordinates_changed.emit(lat, lng)
        elif self.click_count == 1:
            self.end_marker = folium.Marker(
                location=[lat, lng],
                draggable=True,
                icon=folium.Icon(color='red'),
                popup=f"End: ({lat}, {lng})"
            )
            self.end_marker.add_to(self.map)
            self.click_count += 1
            self.end_coordinates_changed.emit(lat, lng)

        self.save_map()
        self.web_view.setUrl(QUrl.fromLocalFile(self.map_path))

    def get_start_coordinates(self):
        if self.start_marker:
            return self.start_marker.location
        else:
            return None

    def get_end_coordinates(self):
        if self.end_marker:
            return self.end_marker.location
        else:
            return None
        
    def handle_marker_dragged(self, marker_type, lat, lng):
        if marker_type == "start":
            self.start_marker.location = [lat, lng]
            self.start_coordinates_changed.emit(lat, lng)
        elif marker_type == "end":
            self.end_marker.location = [lat, lng]
            self.end_coordinates_changed.emit(lat, lng)

        self.save_map()

    def clear_markers(self):
        self.map = folium.Map(location=[1.3521, 103.8198], zoom_start=12)
        self.start_marker = None
        self.end_marker = None
        self.click_count = 0
        self.save_map()
        self.web_view.setUrl(QUrl.fromLocalFile(self.map_path))
        self.clear_button_pressed.emit("")
        

    def save_map(self):
        map_html = self.map.get_root().render()
        custom_js = """
        <script
            type="text/javascript"
            src="qrc:///qtwebchannel/qwebchannel.js"
        ></script>
        <script>
        document.addEventListener('DOMContentLoaded', function() {
            var map_manager = null;
            function onMapClick(e) {
                var data = JSON.stringify(e.latlng)
                map_manager.receive_data("click", data);
            }
            new QWebChannel(qt.webChannelTransport, function (channel) {
                map_manager = channel.objects.map_manager;
            });
            var map_id = document.getElementsByClassName('folium-map')[0].id;
            var lmap = window[map_id];
            lmap.addEventListener("click", onMapClick);

            function addDragEndListener(marker, type) {
                marker.addEventListener("dragend", function(e) {
                    var data = JSON.stringify({
                        type: type,
                        lat: e.target.getLatLng().lat,
                        lng: e.target.getLatLng().lng
                    });
                    map_manager.receive_data("dragend", data);
                });
            }
            lmap.eachLayer(function(layer) {
                if (layer instanceof L.Marker) {
                    if (layer.options.icon.options.markerColor === 'blue') {
                        addDragEndListener(layer, 'start');
                    } else if (layer.options.icon.options.markerColor === 'red') {
                        addDragEndListener(layer, 'end');
                    }
                }
            });
        });
        </script>
        """
        with open(self.map_path, "w") as f:
            f.write(map_html + custom_js)

class RoutePlannerApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Route Planner")
        self.mode_var = 'car'
        self.start_var = ''
        self.end_var = ''

        self.mrt_stations = self.load_mrt_stations()

        self.initUI()

    def load_mrt_stations(self):
        mrt_stations_file = './MRT_Stations.csv'
        mrt_stations_df = pd.read_csv(mrt_stations_file)
        return mrt_stations_df['STN_NAME'].tolist()

    def initUI(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout()

        mode_layout = QHBoxLayout()
        mode_label = QLabel("Select Mode of Transportation:")
        mode_layout.addWidget(mode_label)
        modes = ['Car', 'Bike', 'Walking', 'MRT', 'Multi-Transport']
        self.mode_buttons = {}
        for index, mode in enumerate(modes):
            mode_button = QRadioButton(mode)
            if index == 0:
                mode_button.setChecked(True)
            mode_button.toggled.connect(lambda checked, m=mode.lower(): self.update_inputs(m))
            mode_layout.addWidget(mode_button)
            self.mode_buttons[mode.lower()] = mode_button

        main_layout.addLayout(mode_layout)

        start_layout = QHBoxLayout()
        start_label = QLabel("Start Point:")
        self.start_entry = QLineEdit()
        self.start_dropdown = QComboBox()
        self.start_dropdown.addItems(self.mrt_stations)
        self.start_dropdown.setVisible(False)
        start_layout.addWidget(start_label)
        start_layout.addWidget(self.start_entry)
        start_layout.addWidget(self.start_dropdown)
        main_layout.addLayout(start_layout)

        end_layout = QHBoxLayout()
        end_label = QLabel("End Point:")
        self.end_entry = QLineEdit()
        self.end_dropdown = QComboBox()
        self.end_dropdown.addItems(self.mrt_stations)
        self.end_dropdown.setVisible(False)
        end_layout.addWidget(end_label)
        end_layout.addWidget(self.end_entry)
        end_layout.addWidget(self.end_dropdown)
        main_layout.addLayout(end_layout)

        self.map_view = MapView()
        main_layout.addWidget(self.map_view)
        self.map_view.start_coordinates_changed.connect(self.update_start_entry)
        self.map_view.end_coordinates_changed.connect(self.update_end_entry)
        self.map_view.clear_button_pressed.connect(self.clear_coords)

        self.route = EmbedMap()
        main_layout.addWidget(self.route)

        self.button_layout = QHBoxLayout()
        self.shortest_route_button = QPushButton("Shortest Route")
        self.alternate_route1_button = QPushButton("Alternate Route 1")
        self.alternate_route2_button = QPushButton("Alternate Route 2")

        self.shortest_route_button.clicked.connect(lambda: self.change_route(0))
        self.alternate_route1_button.clicked.connect(lambda: self.change_route(1))
        self.alternate_route2_button.clicked.connect(lambda: self.change_route(2))

        self.button_layout.addWidget(self.shortest_route_button)
        self.button_layout.addWidget(self.alternate_route1_button)
        self.button_layout.addWidget(self.alternate_route2_button)
        self.button_layout_widget = QWidget()
        self.button_layout_widget.setLayout(self.button_layout)
        main_layout.addWidget(self.button_layout_widget)

        self.button_layout_widget.hide()  

        run_button = QPushButton("Generate Route")
        run_button.clicked.connect(self.run_script)
        main_layout.addWidget(run_button)
        main_widget.setLayout(main_layout)

    @pyqtSlot(str)
    def clear_coords(self, string):
        self.start_entry.setText(string)
        self.end_entry.setText(string)

    @pyqtSlot(float, float)
    def update_start_entry(self, lat, lng):
        self.start_entry.setText(f"{lat}, {lng}")

    @pyqtSlot(float, float)
    def update_end_entry(self, lat, lng):
        self.end_entry.setText(f"{lat}, {lng}")

    def update_inputs(self, mode):
        self.mode_var = mode
        if mode == 'mrt':
            self.start_entry.setVisible(False)
            self.end_entry.setVisible(False)
            self.start_dropdown.setVisible(True)
            self.end_dropdown.setVisible(True)
            self.map_view.setVisible(False)
        else:
            self.start_entry.setVisible(True)
            self.end_entry.setVisible(True)
            self.start_dropdown.setVisible(False)
            self.end_dropdown.setVisible(False)
            self.map_view.setVisible(True)

    def run_script(self):
        mode = self.mode_var
        start = self.start_entry.text() if mode != 'mrt' else self.start_dropdown.currentText()
        end = self.end_entry.text() if mode != 'mrt' else self.end_dropdown.currentText()

        if mode == 'mrt':
            if not start or not end:
                QMessageBox.critical(self, "Input Error", "Please select valid MRT stations.")
                return
            script = 'mrt_pathfinder_with_alt.py'
            args = [script, start, end]
        else:
            if not start or not end:
                QMessageBox.critical(self, "Input Error", "Please set valid start and end points on the map.")
                return
            script = 'drive_bike_walk_pathfinder.py' if mode in ['car', 'bike', 'walking'] else 'multi_transport_pathfinder.py'
            start_coords = start.split(',')
            end_coords = end.split(',')
            args = [script, start_coords[0].strip(), start_coords[1].strip(), end_coords[0].strip(), end_coords[1].strip()]

        try:
            result = subprocess.run(['python'] + args, check=True, capture_output=True, text=True)
            output = result.stdout.strip().split('\n')
            if not output:
                QMessageBox.critical(self, "Execution Error", "No output file generated.")
                return
            
            emissions_output = [line for line in output if 'Emissions:' in line]
            self.emissions_list = [f"{float(line.split(': ')[1].replace(' gCO2', '')):.3f} gCO2" for line in emissions_output] if emissions_output else ['N/A']
            filtered_output = self.filter_files_by_mode(output, mode)
            self.display_routes(filtered_output)
        except subprocess.CalledProcessError as e:
            QMessageBox.critical(self, "Execution Error", f"An error occurred while running the script:\n{e.stderr}")

    def display_html(self, html_file):
        self.map_view.setUrl(QUrl.fromLocalFile(html_file))
        self.map_view.setVisible(True)
    
    def filter_files_by_mode(self, files, mode):
        mode_prefix = {
            'car': 'drive',
            'bike': 'bike',
            'walking': 'walk'
        }.get(mode, '')

        return [f for f in files if f.startswith(mode_prefix) and f.endswith('.html')]

    def display_routes(self, route_files):
        self.route_files = route_files

        if route_files:
            self.route.load_map(route_files[0])  # Display first route by default
            self.route.update_emissions(self.emissions_list[0])  
            self.button_layout_widget.show()  

    def change_route(self, index):
        if 0 <= index < len(self.route_files):
            self.route.load_map(self.route_files[index])
            self.route.update_emissions(self.emissions_list[index])  

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RoutePlannerApp()
    window.show()
    sys.exit(app.exec_())
