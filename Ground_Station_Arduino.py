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
import serial.tools.list_ports
import time


class GroundStationArduino:

    def __init__(self, com_port, baudrate):
        self.com_port = serial.Serial(port=com_port, baudrate=baudrate, timeout=.1)
        self.coordinates = []
        self.attempt_number = 0
        return

    def move_position(self, azimuth, elevation):
        # takes in azimuth and elevation to move to
        # sends position to arduino
        position_command = "M" + str(azimuth) + "," + str(elevation)
        print(position_command)
        self.com_port.write(bytes(position_command, "utf-8"))
        time.sleep(.05)
        return

    def adjust_tilt_up(self, degrees):
        # sends command to arduino to move tilt up
        message = "W" + str(degrees)
        try:
            self.com_port.write(bytes(message, "utf-8"))
        except Exception as e:
            print(f"Serial write error: {e}")
        print(message)
        time.sleep(.05)
        return

    def adjust_tilt_down(self, degrees):
        # sends command to arduino to move tilt down
        message = "S" + str(degrees)
        try:
            self.com_port.write(bytes(message, "utf-8"))
        except Exception as e:
            print(f"Serial write error: {e}")
        time.sleep(.05)
        return

    def adjust_pan_positive(self, degrees):
        # sends command to arduino to adjust pan positive
        message = "A" + str(degrees)
        try:
            self.com_port.write(bytes(message, "utf-8"))
        except Exception as e:
            print(f"Serial write error: {e}")
        time.sleep(.05)
        return

    def adjust_pan_negative(self, degrees):
        # sends command to arduino to adjust pan negative
        message = "D" + str(degrees)
        try:
            self.com_port.write(bytes(message, "utf-8"))
        except Exception as e:
            print(f"Serial write error: {e}")
        time.sleep(.05)
        return

    def calibrate(self, starting_azimuth, starting_elevation):
        # sends the arduino the initial starting position (azimuth and elevation) of the ground station
        starting_position = "C" + str(starting_azimuth) + "," + str(starting_elevation)
        self.com_port.write(bytes(starting_position, "utf-8"))
        time.sleep(.05)
        return

    def send_emergency_stop(self):
        self.com_port.write(bytes("E", "utf-8"))
        time.sleep(.1)
        return

    def warm_start(self):
        # requests the GPS position of the ground station
        self.coordinates = self.request_gps_location()
        return self.coordinates

    def request_gps_location(self):
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

    def print_gps_coordinates(self):
        # prints the gps coordinates of the ground station after a request to grab the position
        self.coordinates = self.request_gps_location()
        if len(self.coordinates) > 0:
            print("[{},{},{}]".format(self.coordinates[0], self.coordinates[1], self.coordinates[2]))
        return
