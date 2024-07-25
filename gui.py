import os
import sys
import json

from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QRadioButton, QLineEdit, QComboBox, QPushButton, QMessageBox, QSizePolicy
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtCore import QUrl, pyqtSignal, QObject, QThread, pyqtSlot
from PyQt5.QtWebChannel import QWebChannel
import folium

import pandas as pd
import subprocess
import requests

class ScriptRunner(QThread):
    finished = pyqtSignal(str, list)
    error = pyqtSignal(str)

    def __init__(self, script, args):
        super().__init__()
        self.script = script
        self.args = args

    def run(self):
        try:
            result = subprocess.run([sys.executable] + self.args, check=True, capture_output=True, text=True)
            output = result.stdout.strip().split('\n')
            self.finished.emit(self.script, output)
        except subprocess.CalledProcessError as e:
            self.error.emit(f"An error occurred while running the script:\n{e.stderr}")

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
        self.map_label = QLabel("Generated Map: ")
        layout.addWidget(self.map_label)

        self.emissions_label = QLabel("Carbon Emissions: ")
        layout.addWidget(self.emissions_label)

        self.web_view = QWebEngineView()
        self.web_view.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.web_view.setUrl(QUrl.fromLocalFile(self.map_path))
        layout.addWidget(self.web_view)
    
    def load_map(self, map_path):
        if os.path.exists(map_path):
            self.map_path = os.path.abspath(map_path)
            self.web_view.setUrl(QUrl.fromLocalFile(self.map_path))
            self.map_file = os.path.basename(self.map_path)
            self.map_label.setText("Generated Map: " + self.map_file)

    def update_emissions(self, emissions):
        self.emissions_label.setText(f"Carbon Emissions: {emissions}")

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

        self.map = folium.Map(location=[1.3521, 103.8198], zoom_start=12)

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

    def update_map(self):
        if self.start_marker:
            folium.Marker(
                location=self.start_marker.location,
                draggable=True,
                icon=folium.Icon(color='blue'),
                popup=f"Start: ({self.start_marker.location[0]}, {self.start_marker.location[1]})"
            ).add_to(self.map)
        if self.end_marker:
            folium.Marker(
                location=self.end_marker.location,
                draggable=True,
                icon=folium.Icon(color='red'),
                popup=f"End: ({self.end_marker.location[0]}, self.end_marker.location[1])"
            ).add_to(self.map)

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
        self.setWindowTitle("CommuGreen")
        self.mode_var = 'car'
        self.start_var = ''
        self.end_var = ''
        self.fin_run = 0
        # 0 is initial not running
        # 1 is running
        # 2 is finished
        self.mrt_stations = self.load_mrt_stations()
        self.initUI()

    def load_mrt_stations(self):
        mrt_stations_file = './MRT_Stations.csv'
        mrt_stations_df = pd.read_csv(mrt_stations_file)
        mrt_stations = mrt_stations_df['STN_NAME'].tolist()
        mrt_stations.sort()
        return mrt_stations

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
        self.start_label = QLabel("Start Coordinate:")
        self.start_entry = QLineEdit()
        self.start_entry.setReadOnly(True)
        self.start_entry.setStyleSheet("QLineEdit { background: transparent; border: none; }")
        self.start_dropdown = QComboBox()
        self.start_dropdown.addItems(self.mrt_stations)
        self.start_dropdown.setVisible(False)
        start_layout.addWidget(self.start_label)
        start_layout.addWidget(self.start_entry)
        start_layout.addWidget(self.start_dropdown)
        main_layout.addLayout(start_layout)

        end_layout = QHBoxLayout()
        self.end_label = QLabel("End Coordinate:")
        self.end_entry = QLineEdit()
        self.end_entry.setReadOnly(True)
        self.end_entry.setStyleSheet("QLineEdit { background: transparent; border: none; }")
        self.end_dropdown = QComboBox()
        self.end_dropdown.addItems(self.mrt_stations)
        self.end_dropdown.setVisible(False)
        end_layout.addWidget(self.end_label)
        end_layout.addWidget(self.end_entry)
        end_layout.addWidget(self.end_dropdown)
        main_layout.addLayout(end_layout)

        start_postal_layout = QHBoxLayout()
        self.start_postal_label = QLabel("Start Address/Postal Code:")
        self.start_postal_entry = QLineEdit()
        start_postal_layout.addWidget(self.start_postal_label)
        start_postal_layout.addWidget(self.start_postal_entry)
        main_layout.addLayout(start_postal_layout)

        end_postal_layout = QHBoxLayout()
        self.end_postal_label = QLabel("End Address/Postal Code:")
        self.end_postal_entry = QLineEdit()
        end_postal_layout.addWidget(self.end_postal_label)
        end_postal_layout.addWidget(self.end_postal_entry)
        main_layout.addLayout(end_postal_layout)
        main_layout.addStretch(1)

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

        self.button_layout_widget.setVisible(False)
        self.route.setVisible(False)

        self.run_button = QPushButton("Generate Route")
        self.run_button.clicked.connect(self.run_script)
        main_layout.addWidget(self.run_button)
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
            self.start_postal_label.setVisible(False)
            self.start_postal_entry.setVisible(False)
            self.end_postal_label.setVisible(False)
            self.end_postal_entry.setVisible(False)
            self.start_label.setText("Start MRT Station:")
            self.end_label.setText("End MRT Station:")
        else:
            self.start_entry.setVisible(True)
            self.end_entry.setVisible(True)
            self.start_dropdown.setVisible(False)
            self.end_dropdown.setVisible(False)
            if self.fin_run == 0:
                self.map_view.setVisible(True)
            self.start_postal_label.setVisible(True)
            self.start_postal_entry.setVisible(True)
            self.end_postal_label.setVisible(True)
            self.end_postal_entry.setVisible(True)
            self.start_label.setText("Start Coordinate:")
            self.end_label.setText("End Coordinate:")

    def validate_coordinates(self, coord):
        try:
            lat, lng = map(float, coord.split(','))
            if -90 <= lat <= 90 and -180 <= lng <= 180:
                return True
            else:
                return False
        except ValueError:
            return False

    def geocode_address(self, address):
        try:
            api_key = 'pk.d39a28854a05c1f9b1db56b54ebb6096'
            viewbox = "103.6,1.1,104.1,1.5"
            api_url = (
                f"https://us1.locationiq.com/v1/search.php?key={api_key}&q={address}"
                f"&country=SG&viewbox={viewbox}&bounded=1&format=json"
            )
            response = requests.get(api_url)
            response.raise_for_status()
            data = response.json()
            if data:
                location = data[0]
                return location['lat'], location['lon']
            else:
                raise ValueError("No results found")
            
        except requests.exceptions.HTTPError as http_err:
            if response.status_code == 404:
                print("Invalid postal codes. Please verify again, or use street address instead.")
            else:
                QMessageBox.critical(self, "Geocoding Error", f"HTTP error occurred: {http_err}")
        
        except requests.RequestException as e:
            QMessageBox.critical(self, "Geocoding Error", f"Error occurred while geocoding address/postal code: {address}\n{e}")

        except ValueError as e:
            QMessageBox.warning(self, "Geocoding Error", str(e))

        return None, None
        
    def handle_script_output(self, script, output):
        emissions_output = [line for line in output if 'gCO2' in line]
        if self.script_mode in ['drive', 'bike', 'walk']:
            emissions_output = [item for item in emissions_output if self.script_mode in item.lower()]

        if len(emissions_output) > 3:
            # Find the first string containing "A*" for shortest route
            first_a_star = next((s for s in emissions_output if "A*" in s), None)
            
            # If "A*" string is found
            if first_a_star:
                # Get the last two elements
                last_two_elements = emissions_output[-2:]
                emissions_output = [first_a_star] + last_two_elements
            else:
                # If no "A*" string is found, just take the last three elements
                emissions_output = emissions_output[-3:]
            
        self.emissions_list = [f"{float(line.split(': ')[1].replace(' gCO2', '')):.3f} gCO2" for line in emissions_output]
        while len(self.emissions_list) < 3:
            self.emissions_list.append('N/A')

        filtered_output = self.filter_files_by_mode(output, self.script_mode)
        self.display_routes(filtered_output)

        self.fin_run = 2
        self.map_view.setVisible(False)
        self.run_button.setEnabled(True)
        self.run_button.setText("Start New Route")
        self.route.show()
        self.map_view.web_view.resize(800, 600)
        self.script_runner.deleteLater()

    def handle_script_error(self, error_message):
        self.script_runner.deleteLater()
        self.fin_run = 2
        self.run_button.setEnabled(True)
        self.run_button.setText("Start New Route")
        QMessageBox.critical(self, "Execution Error", error_message)

    def start_new(self):
        # reset to start new generate route
            self.map_view.setVisible(True)
            self.route.setVisible(False)
            self.run_button.setEnabled(True)
            self.button_layout_widget.setVisible(False)
            self.map_view.clear_markers()
            self.end_postal_entry.setText("")
            self.start_postal_entry.setText("")
            self.run_button.setText("Generate Route")
            self.fin_run = 0

    def run_script(self):
        if self.fin_run == 2:
            self.start_new()
            return


        self.run_button.setEnabled(False)
        self.run_button.setText("Generating Route...")
        self.fin_run = 1
        mode = self.mode_var
        start = self.start_entry.text() if mode != 'mrt' else self.start_dropdown.currentText()
        end = self.end_entry.text() if mode != 'mrt' else self.end_dropdown.currentText()

        if mode == 'mrt':
            if not start or not end:
                QMessageBox.critical(self, "Input Error", "Please select valid MRT stations.")
                self.start_new()
                return
            script = 'mrt_pathfinder_with_alt.py'
            args = [script, start, end]
            self.script_mode = 'mrt'
        else:
            if not start or not end:
                if self.start_postal_entry.text() and self.end_postal_entry.text():
                    start_lat, start_lng = self.geocode_address(self.start_postal_entry.text())
                    end_lat, end_lng = self.geocode_address(self.end_postal_entry.text())
                    if start_lat and end_lat:
                        start = f"{start_lat}, {start_lng}"
                        end = f"{end_lat}, {end_lng}"
                    else:
                        QMessageBox.critical(self, "Input Error", "Invalid postal codes. Please verify again, or use street address instead.")
                        self.start_new()
                        return
                else:
                    QMessageBox.critical(self, "Input Error", "Please set valid start and end points on the map.")
                    self.start_new()
                    return
            else:
                if not self.validate_coordinates(start):
                    QMessageBox.critical(self, "Input Error", "Please enter valid start coordinates in the format 'lat, lng'. Alternatively, you may mark the points on the map.")
                    self.start_new()
                    return
                if not self.validate_coordinates(end):
                    QMessageBox.critical(self, "Input Error", "Please enter valid end coordinates in the format 'lat, lng'. Alternatively, you may mark the points on the map.")
                    self.start_new()
                    return

            mode_mapping = {
                'car': 'drive',
                'walking': 'walk'
            }
            self.script_mode = mode_mapping.get(mode, mode)

            start_coords = start.split(',')
            end_coords = end.split(',')

            if self.script_mode in ['drive', 'bike', 'walk']:
                script = 'drive_bike_walk_pathfinder.py'
                args = [script, self.script_mode, start_coords[0].strip(), start_coords[1].strip(), end_coords[0].strip(), end_coords[1].strip()]
            else:
                script = 'multi_transport_pathfinder.py'
                args = [script, start_coords[0].strip(), start_coords[1].strip(), end_coords[0].strip(), end_coords[1].strip()]

        self.script_runner = ScriptRunner(script, args)
        self.script_runner.finished.connect(self.handle_script_output)
        self.script_runner.error.connect(self.handle_script_error)
        self.script_runner.start()

    def display_html(self, html_file):
        self.map_view.setUrl(QUrl.fromLocalFile(html_file))
        self.map_view.setVisible(True)
    
    def filter_files_by_mode(self, files, mode):
        mode_prefix = {
            'drive': 'drive',
            'bike': 'bike',
            'walk': 'walk',
            'mrt': 'mrt',
            'multi_transport': 'multi_transport'
        }.get(mode, '')

        return [f for f in files if f.startswith(mode_prefix) and f.endswith('.html')]

    def display_routes(self, route_files):
        self.route_files = route_files

        if route_files:
            self.route.load_map(route_files[0])
            self.route.update_emissions(self.emissions_list[0])
            self.button_layout_widget.show()

    def change_route(self, index):
        if 0 <= index < len(self.route_files):
            self.route.load_map(self.route_files[index])
            if index < len(self.emissions_list):
                self.route.update_emissions(self.emissions_list[index])
            else:
                self.route.update_emissions('N/A')

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RoutePlannerApp()
    window.show()
    sys.exit(app.exec_())
