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
import requests

class EmbedMap(QMainWindow):
    def __init__(self, map_path=None):
        super().__init__()
        self.map_path = map_path
        # Set up the main window
        self.setWindowTitle("Map Viewer")
        self.setGeometry(100, 100, 800, 600)

        # Set up layout
        layout = QVBoxLayout()
        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

        self.map_file = ""
        self.map_label = QLabel("Generated Map: ")
        layout.addWidget(self.map_label)

        # Set up the web view
        self.web_view = QWebEngineView()
        self.web_view.setUrl(QUrl.fromLocalFile(self.map_path))
        layout.addWidget(self.web_view)
    
    def load_map(self, map_path):
        if os.path.exists(map_path):
            self.map_path = os.path.abspath(map_path)
            self.web_view.setUrl(QUrl.fromLocalFile(self.map_path))
            self.map_file = os.path.basename(self.map_path)
            self.map_label.setText("Generated Map: " + self.map_file)


class MapManager(QObject):
    # manage map clicks, marker dragged etc
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
    # signal to update other class
    start_coordinates_changed = pyqtSignal(float, float)
    end_coordinates_changed = pyqtSignal(float, float)
    clear_button_pressed = pyqtSignal(str)

    def __init__(self):
        super().__init__()

        # Initialize map
        self.map = folium.Map(location=[1.3521, 103.8198], zoom_start=12)  # Singapore coordinates

        # Flags to track clicks and markers
        self.click_count = 0
        self.start_marker = None
        self.end_marker = None

        # location of map.html to pick start and end coords
        self.map_path = os.path.abspath("./map.html")

        # Save map as HTML
        self.save_map()

        # Set up the main window
        self.setWindowTitle("Map Viewer")
        self.setGeometry(100, 100, 800, 600)

        # Set up layout
        layout = QVBoxLayout()
        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

        # Set up the web view
        self.web_view = QWebEngineView()

        # Qwebchannel for python and html file to interact with each other
        channel = QWebChannel(self.web_view)
        map_manager = MapManager(self)
        channel.registerObject("map_manager", map_manager)
        self.web_view.page().setWebChannel(channel)
        
        self.web_view.setUrl(QUrl.fromLocalFile(self.map_path))
        layout.addWidget(self.web_view)

        # Add buttons and labels for getting coordinates and clearing markers
        # self.start_coord_label = QLabel("Start Coordinates: None")
        # self.end_coord_label = QLabel("End Coordinates: None")
        self.clear_button = QPushButton("Clear Markers")
        self.clear_button.clicked.connect(self.clear_markers)

        # layout.addWidget(self.start_coord_label)
        # layout.addWidget(self.end_coord_label)
        layout.addWidget(self.clear_button)

        map_manager.clicked.connect(self.handle_click)
        map_manager.marker_dragged.connect(self.handle_marker_dragged)

    # click event on the map
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
            # self.start_coord_label.setText(f"Start Coordinates: ({lat}, {lng})")
            self.start_coordinates_changed.emit(lat, lng)  # Emit signal
        elif self.click_count == 1:
            self.end_marker = folium.Marker(
                location=[lat, lng],
                draggable=True,
                icon=folium.Icon(color='red'),
                popup=f"End: ({lat}, {lng})"
            )
            self.end_marker.add_to(self.map)
            self.click_count += 1
            # self.end_coord_label.setText(f"End Coordinates: ({lat}, {lng})")
            self.end_coordinates_changed.emit(lat, lng)  # Emit signal

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
            # self.start_coord_label.setText(f"Start Coordinates: ({lat}, {lng})")
            self.start_marker.location = [lat, lng]
            self.start_coordinates_changed.emit(lat, lng)  # Emit signal
        elif marker_type == "end":
            # self.end_coord_label.setText(f"End Coordinates: ({lat}, {lng})")
            self.end_marker.location = [lat, lng]
            self.end_coordinates_changed.emit(lat, lng)  # Emit signal

        # self.update_map()
        self.save_map()
        # self.web_view.setUrl(QUrl.fromLocalFile(self.map_path))

    def update_map(self):
        # self.map = folium.Map(location=[1.3521, 103.8198], zoom_start=12)  # Reset the map
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
                popup=f"End: ({self.end_marker.location[0]}, {self.end_marker.location[1]})"
            ).add_to(self.map)

    def clear_markers(self):
        self.map = folium.Map(location=[1.3521, 103.8198], zoom_start=12)  # Reset the map
        self.start_marker = None
        self.end_marker = None
        self.click_count = 0
        self.save_map()
        self.web_view.setUrl(QUrl.fromLocalFile(self.map_path))
        self.clear_button_pressed.emit("")
        # self.start_coord_label.setText("Start Coordinates: None")
        # self.end_coord_label.setText("End Coordinates: None")
        
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
            // Function to handle click events on the map
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

        # Load MRT stations
        self.mrt_stations = self.load_mrt_stations()

        self.initUI()

    def load_mrt_stations(self):
        mrt_stations_file = './MRT_Stations.csv'
        mrt_stations_df = pd.read_csv(mrt_stations_file)
        mrt_stations = mrt_stations_df['STN_NAME'].tolist()
        mrt_stations.sort()  # Sort stations in alphabetical order
        return mrt_stations

    def initUI(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout()

        # Mode selection
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

        # Start and end coordinates input
        start_layout = QHBoxLayout()
        start_label = QLabel("Start Coordinate:")
        self.start_entry = QLineEdit()
        self.start_dropdown = QComboBox()
        self.start_dropdown.addItems(self.mrt_stations)
        self.start_dropdown.setVisible(False)
        start_layout.addWidget(start_label)
        start_layout.addWidget(self.start_entry)
        start_layout.addWidget(self.start_dropdown)
        main_layout.addLayout(start_layout)

        end_layout = QHBoxLayout()
        end_label = QLabel("End Coordinate:")
        self.end_entry = QLineEdit()
        self.end_dropdown = QComboBox()
        self.end_dropdown.addItems(self.mrt_stations)
        self.end_dropdown.setVisible(False)
        end_layout.addWidget(end_label)
        end_layout.addWidget(self.end_entry)
        end_layout.addWidget(self.end_dropdown)
        main_layout.addLayout(end_layout)

        # Address/Postal code inputs
        start_postal_layout = QHBoxLayout()
        start_postal_label = QLabel("Start Address/Postal Code:")
        self.start_postal_entry = QLineEdit()
        start_postal_layout.addWidget(start_postal_label)
        start_postal_layout.addWidget(self.start_postal_entry)
        main_layout.addLayout(start_postal_layout)

        end_postal_layout = QHBoxLayout()
        end_postal_label = QLabel("End Address/Postal Code:")
        self.end_postal_entry = QLineEdit()
        end_postal_layout.addWidget(end_postal_label)
        end_postal_layout.addWidget(self.end_postal_entry)
        main_layout.addLayout(end_postal_layout)

        # Map view for setting start and end points
        self.map_view = MapView()
        main_layout.addWidget(self.map_view)
        self.map_view.start_coordinates_changed.connect(self.update_start_entry)
        self.map_view.end_coordinates_changed.connect(self.update_end_entry)
        self.map_view.clear_button_pressed.connect(self.clear_coords)

        # Route display textbox
        # self.route_text = QTextEdit()
        # main_layout.addWidget(self.route_text)

        # Route html embed
        self.route = EmbedMap()
        main_layout.addWidget(self.route)
        # self.route.load_map("./singapore_driving_route.html")

        # Run button
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

    def validate_coordinates(self, coord):
        try:
            lat, lng = map(float, coord.split(','))
            if -90 <= lat <= 90 and -180 <= lng <= 180:
                return True
            else:
                return False
        except ValueError:
            return False


    # Convert addresses or postal codes to coordinates using LocationIQ API
    def geocode_address(self, address):
        try: 
            api_key = 'pk.d39a28854a05c1f9b1db56b54ebb6096' # API key from Location IQ
            api_url = f"https://us1.locationiq.com/v1/search.php?key={api_key}&q={address}&country=SG&format=json"
            response = requests.get(api_url)
            response.raise_for_status()  # Raise an exception for HTTP errors
            data = response.json()
            if data:
                location = data[0]
                return location['lat'], location['lon']
            else:
                raise ValueError("No results found")
        except requests.RequestException as e:
            QMessageBox.critical(self, "Geocoding Error", f"Error occurred while geocoding address/postal code: {address}\n{e}")
            return None, None

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
                if self.start_postal_entry.text() and self.end_postal_entry.text():
                    start_lat, start_lng = self.geocode_address(self.start_postal_entry.text())
                    end_lat, end_lng = self.geocode_address(self.end_postal_entry.text())
                    if start_lat and end_lat:
                        start = f"{start_lat}, {start_lng}"
                        end = f"{end_lat}, {end_lng}"
                    else:
                        QMessageBox.critical(self, "Input Error", "Unable to geocode addresses/postal codes.")
                        return
                else:
                    QMessageBox.critical(self, "Input Error", "Please set valid start and end points on the map.")
                    return
            else:
                if not self.validate_coordinates(start):
                    QMessageBox.critical(self, "Input Error", "Please enter valid start coordinates in the format 'lat, lng'. Alternatively, you may mark the points on the map.")
                    return
                if not self.validate_coordinates(end):
                    QMessageBox.critical(self, "Input Error", "Please enter valid end coordinates in the format 'lat, lng'. Alternatively, you may mark the points on the map.")
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
            
            filtered_output = self.filter_files_by_mode(output, mode) # Display relevant files for selected transport mode
            self.display_routes(filtered_output) # Display generated routes
            
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
        # Clear existing layout
        layout = self.route.centralWidget().layout()
        for i in reversed(range(layout.count())):
            widget = layout.itemAt(i).widget()
            if widget is not None:
                widget.deleteLater()

        route_files.sort()

        # Add labels and web views for each route file
        route_labels = ["Shortest Route:", "Alternate Route 1:", "Alternate Route 2:"]
        for idx, route_file in enumerate(route_files):
            if route_file.endswith('.html') and idx < len(route_labels):
                route_label = QLabel(route_labels[idx])
                web_view = QWebEngineView()
                web_view.setUrl(QUrl.fromLocalFile(os.path.abspath(route_file)))
                layout.addWidget(route_label)
                layout.addWidget(web_view)

        self.route.map_label.setText(f"Generated Routes: {', '.join(os.path.basename(f) for f in route_files if f.endswith('.html'))}")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RoutePlannerApp()
    window.show()
    sys.exit(app.exec_())
