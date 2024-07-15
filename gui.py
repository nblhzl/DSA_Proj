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
        self.web_view.setUrl(QUrl.fromLocalFile(self.map_path))
        layout.addWidget(self.web_view)
    
    def load_map(self, map_path):
        if os.path.exists(map_path):
            self.map_path = os.path.abspath(map_path)
            self.web_view.setUrl(QUrl.fromLocalFile(self.map_path))

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
        self.map_path = os.path.abspath("DSA_Proj/map.html")


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
        mrt_stations_file = 'DSA_Proj/MRT_Stations.csv'
        mrt_stations_df = pd.read_csv(mrt_stations_file)
        return mrt_stations_df['STN_NAME'].tolist()

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

        # Start and end points input
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
        self.route.load_map("DSA_Proj/singapore_driving_route.html")

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
            
            # if html exists do the load here
            # self.route.load_map("DSA_Proj/singapore_driving_route.html")


            # old route text code
            # Display routes in the textbox
            # self.route_text.clear()
            # for line in output:
            #     self.route_text.append(line)
            #     if line.endswith('.html'):
            #         self.display_html(line)
        except subprocess.CalledProcessError as e:
            QMessageBox.critical(self, "Execution Error", f"An error occurred while running the script:\n{e.stderr}")

    def display_html(self, html_file):
        self.map_view.setUrl(QUrl.fromLocalFile(html_file))
        self.map_view.setVisible(True)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RoutePlannerApp()
    window.show()
    sys.exit(app.exec_())
