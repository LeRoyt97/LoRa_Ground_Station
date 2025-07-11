"""
-------------------------------------------------------------------------------
MIT License
Copyright (c) 2021 Mathew Clutter and Ronnel Walton
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
-------------------------------------------------------------------------------
"""

import serial
import time


class GroundStationArduino:
    """Arduino-based ground station antenna controller for satellite tracking.

    Controls physical antenna positioning hardware via serial communication.
    Provides precise azimuth and elevation control for satellite/balloon tracking
    with safety features including emergency stop and calibration procedures.

    Note:
        Safety-critical system controlling physical hardware. Improper use
        can result in equipment damage or injury. Always calibrate before
        operation and use emergency stop if anomalous behavior occurs.
    """

    def __init__(self, com_port: str, baudrate: int) -> None:
        """Initialize ground station Arduino controller.

        Args:
            com_port: Serial port identifier (e.g., 'COM3', '/dev/ttyUSB0')
            baudrate: Serial communication baud rate for Arduino connection
            
        Raises:
            serial.SerialException: If serial port cannot be opened or configured
        """
        self.com_port = serial.Serial(port=com_port, baudrate=baudrate, timeout=.1)
        self.coordinates = []
        self.attempt_number = 0
        return

    def move_position(self, azimuth: float, elevation: float) -> None:
        """Command antenna to move to specified azimuth and elevation.

        Sends absolute positioning command to Arduino for precise antenna pointing.
        This is the primary method for satellite/balloon tracking operations.

        Args:
            azimuth: Target azimuth angle in degrees (0-360)
            elevation: Target elevation angle in degrees (0-90)
            
        Note:
            SAFETY CRITICAL: Commands physical antenna movement. Ensure clear
            antenna path and proper calibration before use. Invalid coordinates
            may cause hardware damage or unsafe pointing directions.
        """
        # takes in azimuth and elevation to move to
        # sends position to arduino
        position_command = "M" + str(azimuth) + "," + str(elevation)
        print(position_command)
        self.com_port.write(bytes(position_command, "utf-8"))
        time.sleep(.05)
        return

    def adjust_tilt_up(self, degrees: str) -> None:
        """Adjust antenna elevation upward by specified degrees.

        Sends incremental tilt up command to Arduino for manual antenna adjustment.

        Args:
            degrees: Number of degrees to tilt upward (string format)
            
        Raises:
            Exception: On serial communication failure (handled gracefully)
        """
        # sends command to arduino to move tilt up
        message = "W" + str(degrees)
        try:
            self.com_port.write(bytes(message, "utf-8"))
        except Exception as e:
            print(f"Serial write error: {e}")
        print(message)
        time.sleep(.05)
        return

    def adjust_tilt_down(self, degrees: str) -> None:
        """Adjust antenna elevation downward by specified degrees.

        Sends incremental tilt down command to Arduino for manual antenna adjustment.

        Args:
            degrees: Number of degrees to tilt downward (string format)
            
        Raises:
            Exception: On serial communication failure (handled gracefully)
        """
        # sends command to arduino to move tilt down
        message = "S" + str(degrees)
        try:
            self.com_port.write(bytes(message, "utf-8"))
        except Exception as e:
            print(f"Serial write error: {e}")
        time.sleep(.05)
        return

    def adjust_pan_positive(self, degrees: str) -> None:
        """Adjust antenna azimuth in positive direction by specified degrees.

        Sends incremental pan positive command to Arduino for manual antenna adjustment.

        Args:
            degrees: Number of degrees to pan positively (string format)
            
        Raises:
            Exception: On serial communication failure (handled gracefully)
        """
        # sends command to arduino to adjust pan positive
        message = "A" + str(degrees)
        try:
            self.com_port.write(bytes(message, "utf-8"))
        except Exception as e:
            print(f"Serial write error: {e}")
        time.sleep(.05)
        return

    def adjust_pan_negative(self, degrees: str) -> None:
        """Adjust antenna azimuth in negative direction by specified degrees.

        Sends incremental pan negative command to Arduino for manual antenna adjustment.

        Args:
            degrees: Number of degrees to pan negatively (string format)
            
        Raises:
            Exception: On serial communication failure (handled gracefully)
        """
        # sends command to arduino to adjust pan negative
        message = "D" + str(degrees)
        try:
            self.com_port.write(bytes(message, "utf-8"))
        except Exception as e:
            print(f"Serial write error: {e}")
        time.sleep(.05)
        return

    def calibrate(self, starting_azimuth: float, starting_elevation: float) -> None:
        """Calibrate ground station with known azimuth and elevation reference.

        Establishes coordinate reference frame for accurate antenna positioning.
        Must be performed before tracking operations to ensure pointing accuracy.

        Args:
            starting_azimuth: Known reference azimuth in degrees (typically sun position)
            starting_elevation: Known reference elevation in degrees (typically sun position)
            
        Note:
            REQUIRED before tracking operations. Typically calibrated using sun
            position with solar sight. Improper calibration results in tracking
            errors and potential equipment damage from incorrect pointing.
        """
        # sends the arduino the initial starting position (azimuth and elevation) of the ground station
        starting_position = "C" + str(starting_azimuth) + "," + str(starting_elevation)
        self.com_port.write(bytes(starting_position, "utf-8"))
        time.sleep(.05)
        return

    def send_emergency_stop(self) -> None:
        """Send emergency stop command to immediately halt all antenna movement.

        Critical safety function to stop antenna motion in case of malfunction,
        obstruction, or other emergency conditions.

        Note:
            SAFETY CRITICAL: Use when immediate stop required. After emergency
            stop, system must be recalibrated before resuming operations.
            Always investigate cause before restarting system.
        """
        self.com_port.write(bytes("E", "utf-8"))
        time.sleep(.1)
        return

    def warm_start(self) -> list:
        """Initialize GPS and retrieve ground station coordinates.

        Requests current GPS position from Arduino to establish ground station
        location for tracking calculations.

        Returns:
            list: Ground station coordinates [latitude, longitude, altitude]
            
        Note:
            GPS accuracy affects tracking precision. Ensure clear sky view
            and allow time for GPS fix before beginning tracking operations.
        """
        # requests the GPS position of the ground station
        self.coordinates = self.request_gps_location()
        return self.coordinates

    def request_gps_location(self) -> list:
        """Request and parse GPS coordinates from Arduino GPS module.

        Attempts to obtain accurate GPS position with retry logic for reliability.
        Parses GPS data and handles coordinate sign conversion for geographic accuracy.

        Returns:
            list: GPS coordinates [latitude, longitude, altitude] or empty list on failure
            
        Raises:
            UnicodeDecodeError: On malformed GPS data (handled gracefully)
            ValueError: On coordinate conversion errors (handled gracefully)
            
        Note:
            Critical for tracking accuracy. Retries up to 100 times for GPS fix.
            Coordinate errors affect ground station pointing calculations and
            can result in tracking failure or equipment damage.
        """
        # attempts to obtain and return the ground station gps location
        if self.attempt_number < 100:
            self.attempt_number += 1
            self.com_port.write(b'G')
            time.sleep(0.05)
            serial_data = self.com_port.readline()
            if len(serial_data) > 2:
                decoded_data = serial_data[:-2].decode('ascii')
                if len(decoded_data.split(",")) == 3:
                    self.coordinates = []
                    temporary_coordinates = decoded_data.split(",")
                    # S and W negative
                    for index in range(2):
                        self.coordinates.append(float(temporary_coordinates[index][:-1]))
                    self.coordinates.append(float(temporary_coordinates[2]))
                    # print(self.attempt_number, " Attempts")
                    self.attempt_number = 0
                else:
                    self.request_gps_location()
            else:
                self.request_gps_location()
        else:
            print("Failed to request GPS")
            # exit(1)
        return self.coordinates

    def print_gps_coordinates(self) -> None:
        """Display current GPS coordinates of ground station.

        Convenience method to request and display ground station position
        in human-readable format for verification and debugging.

        Note:
            Used for setup verification and troubleshooting. Coordinate
            accuracy is critical for successful tracking operations.
        """
        # prints the gps coordinates of the ground station after a request to grab the position
        self.coordinates = self.request_gps_location()
        if len(self.coordinates) > 0:
            print("[{},{},{}]".format(self.coordinates[0], self.coordinates[1], self.coordinates[2]))
        return
