import os
import re
from _pyrepl import reader

from lora_reader import LoraReader, LoraDataObject
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.uic import loadUi
import sys
from PyQt5.QtCore import pyqtSlot, pyqtSignal, QObject, QThread
import serial
import serial.tools.list_ports
import time
from ground_station_arduino import GroundStationArduino
from satellite_tracking_math import TrackingMath
from sun_position import sunpos
import datetime
import csv
import statistics
import numpy as np
import matplotlib.pyplot as plt


# todo: LeRoy: Reorganize GUI: Make status_box always visible, so returns/feedback can be seen during setup.
# todo: smarter GS location input (handle both decimal-degrees and degrees-minutes-seconds)
# todo: save serial monitor data to a .txt file (GUI: file --> save sort of thing)
# todo: Clean up unused Debug prints. Some don't have "Debug" in them
# todo: calculate distance from balloon


class MainWindow(QMainWindow):
    log_signal = pyqtSignal(str)

    def __init__(self) -> None:
        """Sets up the GUI interface, initializes state variables, populates
        port selection combo boxes, and connects all button signals to
        their respective handler methods.
        """
        super(MainWindow, self).__init__()
        loadUi("LoRa_Designer.ui", self)

        # === Hardware Interfaces and Threads ===
        self.reader = None
        self.ground_station_arduino = None
        self.track_thread = None
        self.worker = None

        # === Serial Port Management ===
        self.ports = None
        self.lora_port_names = []
        self.arduino_port_names = []

        # === State Flags ===
        self.is_arduino_connected = False
        self.is_calibrated = False
        self.is_ground_station_location_set = False
        self.is_lora_listening = False
        self.is_predicting_track = False
        self.is_tracking = False

        # === Ground Station Parameters ===
        self.GroundStationArduino = None
        self.ground_station_latitude = None
        self.ground_station_longitude = None
        self.ground_station_altitude = None
        self.starting_azimuth = None
        self.starting_elevation = None

        # === Connect ComboBox Refreshes ===
        self.refresh_ports(self.LoRaComboBox, self.lora_port_names, "lora")
        self.refresh_ports(self.ArduinoComboBox, self.arduino_port_names, "arduino")

        # === Connect Serial Port Buttons ===
        self.LoRaRefreshButton.clicked.connect(
            lambda: self.refresh_ports(self.LoRaComboBox, self.lora_port_names, "lora")
        )
        self.LoRaSelectButton.clicked.connect(self.start_lora_reader)
        self.ArduinoRefreshButton.clicked.connect(
            lambda: self.refresh_ports(
                self.ArduinoComboBox, self.arduino_port_names, "arduino"
            )
        )
        self.ArduinoSelectButton.clicked.connect(self.start_arduino)
        self.ClearSerialButton.clicked.connect(self.clear_serial)

        # === Connect Adjustment Buttons ===
        self.UpButton.clicked.connect(self.tilt_up)
        self.DownButton.clicked.connect(self.tilt_down)
        self.ClockWiseButton.clicked.connect(self.pan_counter_clockwise)
        self.CounterClockeWiseButton.clicked.connect(self.pan_clockwise)
        self.log_signal.connect(self.display_data)

        # === Connect SetUp Buttons ===
        self.setGSLocationButton.clicked.connect(self.set_ground_station_location)
        self.calculateStartingPosButton.clicked.connect(self.get_starting_position)
        self.returnToSunButton.clicked.connect(self.return_to_sun)
        self.setStartingPosButton.clicked.connect(self.calibrate)

        # === Connect Start and Stop Buttons ===
        self.startButton.clicked.connect(self.check_if_ready)
        self.stopButton.clicked.connect(self.stop_tracking)
        self.EStopButton.clicked.connect(self.emergency_stop)
        self.predictionStartButton.clicked.connect(self.set_predict_track)

    def refresh_ports(self, combo_box, port_names_list: list, target: str) -> None:
        """Refresh available serial ports in combo box.

        Args:
            combo_box: Qt combo box widget to populate with port descriptions
            port_names_list: List to store corresponding port device names
            target: "arduino" or "lora"
        """
        try:
            if target == "lora" and self.reader:
                self.is_lora_listening = False
                self.reader.stop()
                self.reader.join()
                if self.reader.serial_port.is_open:
                    self.reader.serial_port.close()
                self.reader = None

            elif (
                target == "arduino"
                and self.GroundStationArduino
                and self.GroundStationArduino.com_port.is_open
            ):
                self.GroundStationArduino.com_port.close()
                self.GroundStationArduino = None
                self.is_arduino_connected = False

            combo_box.clear()
            port_names_list.clear()
            ports = serial.tools.list_ports.comports()

            for port_info in sorted(ports, key=lambda p: p.device):
                combo_box.addItem(port_info.description)
                port_names_list.append(port_info.device)
        except Exception as err:
            print(f"Refresh ports error: {err}")
            self.statusBox.append(f"Refresh ports error: {err}")

    def start_lora_reader(self) -> None:
        """Start LoRa reader thread for receiving telemetry data."""
        # print("Select button clicked")
        selected_index = self.LoRaComboBox.currentIndex()

        if not self.is_lora_listening and 0 <= selected_index < len(
            self.lora_port_names
        ):
            selected_port = self.lora_port_names[selected_index]
            self.statusBox.setPlainText(f"LoRa Connecting to {selected_port}")
            try:
                self.reader = LoraReader(
                    port=selected_port, window=self, callback=self.log_signal
                )
                self.reader.start()
                self.is_lora_listening = True
            except Exception as err:
                self.log_signal.emit(f"Error in LoRaReader: {err}")
            # print("LoRaReader thread Started")
        else:
            self.log_signal.emit("No valid LoRa port selected")

    def start_arduino(self) -> None:
        """Initialize and connect to Arduino ground station controller."""
        # checks if arduino is selected, and if the connection is not already made, instantiates an instance of
        # the GroundStationArduino class
        # if an arduino is connected, or one is not selected, the function returns

        selected_index = self.ArduinoComboBox.currentIndex()

        if (
            not self.is_arduino_connected
            and 0 <= selected_index < self.ArduinoComboBox.count()
        ):
            try:
                selected_port = self.arduino_port_names[selected_index]
                self.ground_station_arduino = GroundStationArduino(selected_port, 9600)
                print("After ground_station_arduino init", self.ground_station_arduino)
                self.AdjustmentLogBox.append("Connected to {selected_port}!")
                self.statusBox.append(f"Connected to {selected_port}!")
                self.is_arduino_connected = True
            except Exception as err:
                self.statusBox.append(f"Failed to connect Arduino: {err}")
        elif self.is_arduino_connected:
            self.AdjustmentLogBox.setPlainText("Arduino already connected")
            self.statusBox.append("Arduino already connected")
        else:
            self.statusBox.append("Unable to connect to Arduino")
            self.AdjustmentLogBox.setPlainText("Unable to connect to Arduino")
        return

    def tilt_up(self) -> None:
        """Tilt ground station antenna upward by configured degrees."""
        # if an arduino is connected, uses ground_station_arduino to adjust the tilt up
        if self.is_arduino_connected:
            self.ground_station_arduino.adjust_tilt_up(
                self.degreesPerClickBox.currentText()
            )
            self.AdjustmentLogBox.setPlainText(
                "adjusting tilt up "
                + self.degreesPerClickBox.currentText()
                + " degrees"
            )
        else:
            print("Unable to connect to ground station motors")
            self.AdjustmentLogBox.setPlainText("Not connected to ground station motors")

        return

    def tilt_down(self) -> None:
        """Tilt ground station antenna downward by configured degrees."""
        # if an arduino is connected, uses ground_station_arduino to adjust the tilt down
        if self.is_arduino_connected:
            self.ground_station_arduino.adjust_tilt_down(
                self.degreesPerClickBox.currentText()
            )
            self.AdjustmentLogBox.setPlainText(
                "adjusting tilt down "
                + self.degreesPerClickBox.currentText()
                + " degrees"
            )
        else:
            print("Unable to connect to ground station motors")
            self.AdjustmentLogBox.setPlainText("Not connected to ground station motors")

        return

    def pan_counter_clockwise(self) -> None:
        """Pan ground station antenna counter-clockwise by configured degrees."""
        # if an arduino is connected, uses ground_station_arduino to adjust the pan counter-clockwise
        if self.is_arduino_connected:
            self.ground_station_arduino.adjust_pan_negative(
                self.degreesPerClickBox.currentText()
            )
            self.AdjustmentLogBox.setPlainText(
                "adjusting pan "
                + self.degreesPerClickBox.currentText()
                + " Counter Clockwise"
            )
        else:
            print("Unable to connect to ground station motors")
            self.AdjustmentLogBox.setPlainText("Not connected to ground station motors")

        return

    def pan_clockwise(self) -> None:
        """Pan ground station antenna clockwise by configured degrees."""
        # if an arduino is connected, uses ground_station_arduino to adjust the pan clockwise
        if self.is_arduino_connected:
            self.ground_station_arduino.adjust_pan_positive(
                self.degreesPerClickBox.currentText()
            )
            self.AdjustmentLogBox.setPlainText(
                "adjusting pan " + self.degreesPerClickBox.currentText() + " Clockwise"
            )
        else:
            print("Unable to connect to ground station motors")
            self.AdjustmentLogBox.setPlainText("Not connected to ground station motors")

        return

    def display_data(self, data: str) -> None:
        """Display received data in status box.

        Args:
            data: Data string to display in the status box

        Note:
            Called from LoRa reader thread via signal for thread-safe operation
        """
        # Called from LoraReader thread - must use signal-safe method
        self.statusBox.append(data)

    @staticmethod
    def is_dms_gps(lat_str: str, long_str: str) -> bool:
        """Check if the latitude or longitude strings are in Degree Minute Seconds format
        Will match against:

        optional negative sign
        + 1-3 digits
        + degree symbol or space
        + 1-2 digits
        + apostraphe or space
        + seconds with optional decimal
        + optional quotes or spaces
        + optional direction
        """

        dms_regex = re.compile(
            r"""^              # start of string
            -?                 # optional minus sign
            \d{1,3}            # degrees (1 to 3 digits)
            [°\s]              # degree symbol or space
            \s*                # optional whitespace
            \d{1,2}            # minutes (1 to 2 digits)
            ['\s]              # apostrophe or space
            \s*                # optional whitespace
            \d{1,2}(?:\.\d+)?  # seconds with optional decimal
            ["”\s]?            # optional quote, double-quote, or space
            \s*                # optional whitespace
            [NSEW]?            # optional direction
            $                  # end of string
            """,
            re.VERBOSE | re.IGNORECASE,
        )

        for str in (lat_str, long_str):
            if not dms_regex.fullmatch(
                str
            ):  # fullmatch to protect against prefix/postfix garbage
                return False
        return True

    def convert_dms_to_dg(self, dms_string: str) -> str | None:
        """Converts a Degree Minute Second String matching shape of is_dms_gps,
        To a decimal degree GPS formatted string

        Raises:
            ValueError: If input DMS string doesn't have 3 numbers
        """
        try:
            print(dms_string)
            numbers = re.findall(
                r"""
                    \d+         # match one or more digit
                    (?:\.\d+)?  # match optionally against a . or a digit one or more times
                """,
                dms_string.strip(),
                re.VERBOSE,
            )
            print(numbers)
            direction = re.search(r"[NSEW]", dms_string.strip().upper())

            if len(numbers) < 2 or len(numbers) > 3:
                raise ValueError(
                    "DMS string needs atleast degrees and minutes, but not more than three."
                )

            degrees = float(numbers[0])
            minutes = float(numbers[1])
            seconds = float(numbers[2] if len(numbers) < 2 else 0.0)

            dg = degrees + minutes / 60 + seconds / 3600

            if direction and direction.group() in ("S", "W"):
                dg = -dg

            return str(dg)

        except ValueError as err:
            self.statusBox.append(f"Conversion failed: {err}")
            return None

    def set_ground_station_location(self) -> None:
        """Set ground station GPS coordinates from user input.

        Raises:
            ValueError: If input values cannot be converted to float
        """
        # this ensures that the arduino is connected, and valid text is present in the gs location text boxes
        # if the values present can be converted to floats, the starting location of the gs is set
        try:
            if self.is_arduino_connected:
                latitude_string = self.GSLatBox.text().strip()
                longitude_string = self.GSLongBox.text().strip()
                altitude_string = self.GSAltBox.text().strip()

                if self.is_dms_gps(latitude_string, longitude_string):
                    latitude_string, longitude_string = (
                        self.convert_dms_to_dg(latitude_string),
                        self.convert_dms_to_dg(longitude_string),
                    )

                self.ground_station_latitude = float(latitude_string)
                print(self.ground_station_latitude)

                self.ground_station_longitude = float(longitude_string)
                print(self.ground_station_longitude)

                self.ground_station_altitude = float(altitude_string)
                print(self.ground_station_altitude)

                self.statusBox.append("Ground station location entered successfully!")
                self.is_ground_station_location_set = True

            else:
                self.statusBox.append("Please connect arduino")
                self.is_ground_station_location_set = False
        except ValueError:
            print("numbers only for GPS location (decimal degrees)")
            self.statusBox.append(
                "Invalid GPS location entered. Please only enter numbers"
            )
        except Exception as err:
            self.statusBox.append(f"set_ground_station_location error: {err}")

    def get_starting_position(self) -> None:
        """Calculate sun position for ground station calibration.

        Uses current UTC time and ground station location to determine
        sun's azimuth and elevation for antenna pointing reference.
        Populates starting position text boxes with calculated values.
        """
        # this makes a call to sunposition to calculate the azimuth and elevation of the sun at the current location
        # of the ground station
        # it populates the starting azimuth and elevation boxes
        try:
            if self.is_ground_station_location_set:
                now = datetime.datetime.now(tz=datetime.timezone.utc)
                azimuth, elevation = sunpos(
                    now,
                    self.ground_station_latitude,
                    self.ground_station_longitude,
                    self.ground_station_altitude,
                )[
                    :2
                ]  # discard RA, dec, H

                self.starting_azimuth = azimuth
                self.starting_elevation = elevation

                self.startingAzimuthBox.setText(str(azimuth))
                self.startingElevationBox.setText(str(elevation))

            else:
                self.statusBox.append(
                    "Please set ground station location "
                    "and point at the sun using solar sight"
                )
        except Exception as err:
            self.statusBox.append(f"get_starting_position error: {err}")
            print("get_starting_position error: ", err)

        return

    def calibrate(self) -> None:
        """Calibrate ground station with starting azimuth and elevation values.

        Raises:
            ValueError: If position values cannot be converted to float
        """
        # sends the ground_station_arduino class the starting azimuth and elevation
        if self.is_arduino_connected:
            try:
                starting_azimuth_string = self.startingAzimuthBox.toPlainText().strip()
                starting_azimuth = float(starting_azimuth_string)
                print(starting_azimuth)

                starting_elevation_string = (
                    self.startingElevationBox.toPlainText().strip()
                )
                starting_elevation = float(starting_elevation_string)
                print(starting_elevation)

                self.ground_station_arduino.calibrate(
                    starting_azimuth, starting_elevation
                )
                self.is_calibrated = True
                self.statusBox.append("Successfully calibrated!")
            except ValueError:
                print("numbers only for initial azimuth and elevation")
                self.statusBox.append("Invalid input for initial azimuth and elevation")
            except Exception as err:
                print(f"calibrate error: {err}")
                self.statusBox.append(f"calibrate error: {err}")
        else:
            print("not connected to arduino")
            self.statusBox.append("Not connected to arduino")

        return

    def return_to_sun(self):
        """Point ground station back to current sun position."""
        if (
            self.is_arduino_connected
            and self.is_ground_station_location_set
            and self.is_calibrated
        ):
            now = datetime.datetime.now(tz=datetime.timezone.utc)
            azimuth, elevation = sunpos(
                now,
                self.ground_station_latitude,
                self.ground_station_longitude,
                self.ground_station_altitude,
            )[
                :2
            ]  # discard RA, dec, H

            self.ground_station_arduino.move_position(azimuth, elevation)

            self.starting_azimuth = azimuth
            self.starting_elevation = elevation

            self.startingAzimuthBox.setPlainText(str(self.starting_azimuth))
            self.startingElevationBox.setPlainText(str(self.starting_elevation))
            self.statusBox.append("at new sun position")

        else:
            self.statusBox.append(
                "Ensure that arduino is connected, GS location is set and calibration is set"
            )
            print("Cannot point back at the sun")

        return

    def set_predict_track(self) -> None:
        """Enable predictive tracking mode and check system readiness."""
        # sets the predict track bool variable
        # then calls the check_if_ready function to ensure all conditions to track have been met
        self.is_predicting_track = True
        self.check_if_ready()
        return

    def check_if_ready(self) -> bool:
        """Verify all systems are ready for tracking operation.

        Returns:
            True if ready to track and tracking started, False otherwise
        """
        # this function ensures that all conditions to track have been met
        # if they have been, it calls the appropriate function to either start tracking with/without predictions
        try:
            if self.is_calibrated:
                print("Calibrated!")
                # self.statusBox.append("Calibrated!")
            else:
                print("starting position not set")
                self.statusBox.append("Please set staring azimuth and elevation")

            if self.is_ground_station_location_set:
                print("Ground station location set!")
            else:
                print("Ground Station Location not assigned")
                self.statusBox.append("Ground Station location not assigned")

            if self.is_lora_listening:
                print("LoRa is listening!")
            else:
                print("LoRa is not listening!")
                self.statusBox.append("LoRa is not listening!")

            if self.is_arduino_connected:
                print("Arduino connected!")
            else:
                print("Please connect to the Arduino")
                self.statusBox.append("Please connect to the Arduino")

            print("\n")

            if (
                self.is_arduino_connected
                and self.is_lora_listening
                and self.is_calibrated
                and self.is_ground_station_location_set
            ):

                if self.is_predicting_track:
                    self.statusBox.append("Starting tracking with predictions!")
                    self.call_predict_track()
                else:
                    self.statusBox.append("Starting tracking!")
                    print("starting tracking!")
                    self.call_track()
                    return True
            else:
                print("not ready to track yet")
                return False
        except Exception as err:
            print(f"check_if_ready error: {err}")

    def call_track(self) -> None:
        """Start basic tracking thread without prediction.

        Creates and starts worker thread for standard balloon tracking.
        """
        # sets up the qt thread to start tracking, and starts the thread
        try:
            self.is_tracking = True
            self.statusBox.append("Tracking!")
            self.track_thread = QThread()
            self.worker = Worker(self.reader)

            self.worker.moveToThread(self.track_thread)

            self.track_thread.started.connect(self.worker.track)

            self.worker.finished.connect(
                self.track_thread.quit
            )  # pycharm has bug, this is correct
            self.worker.finished.connect(
                self.worker.deleteLater
            )  # https://youtrack.jetbrains.com/issue/PY-24183?_ga=2.240219907.1479555738.1625151876-2014881275.1622661488
            self.track_thread.finished.connect(self.track_thread.deleteLater)

            self.startButton.setEnabled(False)
            self.predictionStartButton.setEnabled(False)
            self.setStartingPosButton.setEnabled(False)

            self.track_thread.start()
        except Exception as err:
            print(f"call_track error: {err}")

    def call_predict_track(self) -> None:
        """Start tracking thread with predictive capabilities.

        Creates and starts worker thread for predictive balloon tracking.
        """
        # sets up the qt thread to start tracking with predictions and starts the thread
        self.statusBox.append("Tracking with predictions!")
        print("In predict_track call")
        self.is_tracking = True
        self.track_thread = QThread()
        self.worker = Worker(self.reader)

        self.worker.moveToThread(self.track_thread)

        self.track_thread.started.connect(self.worker.predict_track)

        self.worker.finished.connect(
            self.track_thread.quit
        )  # pycharm has bug, this is correct
        self.worker.finished.connect(
            self.worker.deleteLater
        )  # https://youtrack.jetbrains.com/issue/PY-24183?_ga=2.240219907.1479555738.1625151876-2014881275.1622661488
        self.track_thread.finished.connect(self.track_thread.deleteLater)

        self.startButton.setEnabled(False)
        self.predictionStartButton.setEnabled(False)
        self.setStartingPosButton.setEnabled(False)

        self.track_thread.start()

    def stop_tracking(self) -> None:
        """Stop current tracking operation and reset UI controls."""
        # this stops the tracking thread, thus stopping the tracking
        if self.is_tracking:
            self.is_tracking = False
            self.is_predicting_track = False
            self.startButton.setEnabled(True)
            self.predictionStartButton.setEnabled(True)
            self.setStartingPosButton.setEnabled(True)
            self.statusBox.append("tracking stopped")
        return

    def emergency_stop(self) -> None:
        """Execute emergency stop procedure for ground station."""
        if self.is_arduino_connected:
            self.ground_station_arduino.send_emergency_stop()
            self.stop_tracking()

            self.statusBox.append(
                "E-Stop triggered \n Please recalibrate before starting again"
            )
            print("E-Stopped must recalibrate before starting tracking")

        return

    def display_calculations(
        self, distance: float, azimuth: float, elevation: float
    ) -> None:
        """Display tracking calculations on GUI.

        Args:
            distance: Distance to target in meters
            azimuth: Target azimuth in degrees
            elevation: Target elevation in degrees
        """
        # this displays the outputs from the tracking threads on the GUI
        self.distanceDisplay.setPlainText(str(distance))
        self.azimuthDisplay.setPlainText(str(azimuth))
        self.elevationDisplay.setPlainText(str(elevation))
        return

    def closeEvent(self, event) -> None:
        """Handle application close event and cleanup resources.

        Args:
            event: Qt close event object

        Note:
            Stops LoRa reader thread if running before accepting close event
        """
        if self.reader:
            self.reader.stop()
            self.reader.join()
        event.accept()

    def clear_serial(self) -> None:
        """Clear the serial status display box."""
        self.statusBox.clear()


class Worker(QObject):
    # worker class to track without making the GUI hang
    finished = pyqtSignal()
    calculation_signal = pyqtSignal(float, float, float)

    def __init__(self, reader) -> None:
        """Initialize worker thread for tracking operations.

        Args:
            reader: LoRa reader instance for telemetry data reception
        """
        super().__init__()
        self.reader = reader
        self.iteration_count = 0

    def track(self) -> None:
        """Execute basic tracking algorithm.

        Monitors balloon position every 5 seconds and calculates
        required antenna pointing angles for tracking. Sends movement
        commands to ground station when new position data is received.

        Note:
            Runs in separate thread to prevent GUI blocking
        """
        # basic tracking algorithm
        # checks for updated position every 5 seconds
        # if a new position has been found, calculate the azimuth and elevation to point at the new location
        # send the motors a command to move to the new position

        print("LoRa data:", self.reader.data)
        print("ground_station_arduino:", main_window.ground_station_arduino)

        timer = time.time() - 4.0
        try:
            while main_window.is_tracking:

                if (time.time() - timer) > 5.0:
                    timer = time.time()
                    balloon_coordinates = [
                        self.reader.data.latitude,
                        self.reader.data.longitude,
                        self.reader.data.altitude,
                    ]
                    if not balloon_coordinates:
                        pass
                    else:
                        # note that TrackingMath takes arguments as lat, long, altitude
                        tracking_calculation = TrackingMath(
                            main_window.ground_station_latitude,
                            main_window.ground_station_longitude,
                            main_window.ground_station_altitude,
                            *balloon_coordinates,
                        )

                        distance = tracking_calculation.distance
                        new_elevation = tracking_calculation.elevation()
                        new_azimuth = tracking_calculation.azimuth()

                        print(
                            str(self.iteration_count)
                            + " Distance "
                            + str(distance)
                            + " Azimuth: "
                            + str(new_azimuth)
                            + ", Elevation: "
                            + str(new_elevation)
                        )

                        self.calculation_signal.connect(
                            main_window.display_calculations
                        )  # this seems to happen a lot for some reason
                        self.calculation_signal.emit(
                            distance, new_azimuth, new_elevation
                        )

                        main_window.ground_station_arduino.move_position(
                            new_azimuth, new_elevation
                        )

                        self.iteration_count += 1

            print("All done!")
            self.finished.emit()  # same pycharm bug as above
            return
        except Exception as err:
            print(f"error in tracking while {err}")

    def predict_track(self) -> None:
        """Execute predictive tracking algorithm.

        Uses motion prediction to anticipate balloon position and
        maintain tracking during communication gaps. Calculates
        velocity vectors and predicts future positions when no
        new telemetry data is received.

        Note:
            Saves tracking data to CSV file for analysis
        """
        # check for new location from server
        # if the new location is still the same, go to prediction
        # if there is new location, go to that latest location

        # find the difference between the latest lat/long location and the one before the last one and time
        # using last vertical velocity/altitude, find difference between altitudes/new altitude after ~1 second
        # find the amount that the position is changing each ~second
        # input the new predicted lat/long/alt into math equations to get new azimuth/elevation
        # go to the predicted elevation/azimuth

        print("In predict_track")

        timer = time.time()
        newest_location = [
            self.reader.data.latitude,
            self.reader.data.longitude,
            self.reader.data.altitude,
        ]
        old_location = [
            self.reader.data.latitude,
            self.reader.data.longitude,
            self.reader.data.altitude,
        ]
        prediction_step = 1

        calculations = open("predictedOutput.csv", "w")
        csv_writer = csv.writer(calculations)
        calculation_fields = ["Distance", "Azimuth", "Elevation", "r/p"]
        csv_writer.writerow(calculation_fields)

        azimuth_list = []
        elevation_list = []

        while main_window.is_predicting_track:
            if (time.time() - timer) > 1:
                timer = time.time()
                current_data = [
                    self.reader.data.latitude,
                    self.reader.data.longitude,
                    self.reader.data.altitude,
                ]

                if newest_location == current_data:
                    # need to predict!
                    print("predicted output")
                    time_delta = main_window.balloon.get_time_difference()
                    print("The time delta is: " + str(time_delta))
                    latitude_step = (newest_location[0] - old_location[0]) / time_delta
                    longitude_step = (newest_location[1] - old_location[1]) / time_delta
                    altitude_step = (newest_location[2] - old_location[2]) / time_delta

                    tracking_calculation = TrackingMath(
                        main_window.ground_station_latitude,
                        main_window.ground_station_longitude,
                        main_window.ground_station_altitude,
                        newest_location[1] + (prediction_step * longitude_step),
                        newest_location[0] + (prediction_step * latitude_step),
                        newest_location[2] + (prediction_step * altitude_step),
                    )

                    distance = tracking_calculation.distance
                    new_elevation = tracking_calculation.elevation()
                    new_azimuth = tracking_calculation.azimuth()

                    elevation_list.append(new_elevation)
                    azimuth_list.append(new_azimuth)

                    # keep average of azimuth/elevations
                    # if new calculation is outlier, throw it out, don't go to new spot
                    # reset average between pings
                    # alternatively, implement some type of filter (savitzky golay, kalman, etc.)

                    """
                    if new_elevation > np.mean(elevation_list) + (2 * np.std(elevation_list)) or new_elevation < np.mean(elevation_list) - (2 * np.std(elevation_list)) \
                            or new_azimuth > np.mean(azimuth_list) + (2 * np.std(azimuth_list)) or new_azimuth < np.mean(azimuth_list) - (2 * np.std(azimuth_list)):
                        print("outlier detected! ")
                        pass
                    else:
                        print("distance: " + str(distance))
                        print("elevation: " + str(new_elevation))
                        print("azimuth: " + str(new_azimuth) + "\n")
                    """
                    self.calculation_signal.connect(main_window.display_calculations)
                    self.calculation_signal.emit(distance, new_azimuth, new_elevation)

                    row = [distance, new_azimuth, new_elevation, "p"]
                    csv_writer.writerow(row)

                    main_window.ground_station_arduino.move_position(
                        new_azimuth, new_elevation
                    )

                    prediction_step += 1

                else:
                    # go to the new actual spot
                    old_location = newest_location
                    newest_location = current_data

                    # note that TrackingMath takes arguments as lat, long, alt
                    tracking_calculation = TrackingMath(
                        main_window.ground_station_latitude,
                        main_window.ground_station_longitude,
                        main_window.ground_station_altitude,
                        *current_data,
                    )

                    distance = tracking_calculation.distance
                    new_elevation = tracking_calculation.elevation()
                    new_azimuth = tracking_calculation.azimuth()

                    self.calculation_signal.connect(
                        main_window.display_calculations
                    )  # this seems to happen a lot for some reason
                    self.calculation_signal.emit(distance, new_azimuth, new_elevation)

                    print("Got new real ping!")
                    print("distance: " + str(distance))
                    print("elevation: " + str(new_elevation))
                    print("azimuth: " + str(new_azimuth) + "\n")

                    main_window.ground_station_arduino.move_position(
                        new_azimuth, new_elevation
                    )

                    row = [distance, new_azimuth, new_elevation, "r"]
                    csv_writer.writerow(row)

                    prediction_step = 1
                    azimuth_list = []
                    elevation_list = []

        print("All done tracking with predictions! :)")
        calculations.close()
        self.finished.emit()
        return


if __name__ == "__main__":
    os.environ["QT_AUTO_SCREEN_SCALE_FACTOR"] = "1"
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.showMaximized()
    app.exec_()
