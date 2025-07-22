import copy
import dataclasses
import re
from difflib import Match

from typing import Optional, Callable, TYPE_CHECKING

if TYPE_CHECKING:
    from main import MainWindow

import serial
import threading


@dataclasses.dataclass
class LoraDataObject:
    """Data container for LoRa telemetry information.

    Represents a complete telemetry packet received from a LoRa transmitter,
    containing position data and transmission metadata for balloon tracking.

    Attributes:
        malformed: Bool indicating if packet contains invalid chars
        raw_lora_string: The passed LoRa string
        identifier_one: Primary identifier string for the transmitter
        latitude: Geographic latitude in decimal degrees (-90 to +90)
        longitude: Geographic longitude in decimal degrees (-180 to +180)
        altitude: Altitude above sea level in meters
        last_sent: Timestamp or sequence number of last transmission sent
        last_complete: Timestamp or sequence number of last complete transmission
        identifier_two: Secondary identifier string for validation
        rssi: Received Signal Strength Indicator in dBm
        snr: LoRa Signal to Noise Ratio in dB

    Note:
        Used for safety-critical balloon tracking operations. Coordinate
        accuracy is essential for ground station pointing calculations.
    """

    malformed: bool = False
    raw_lora_string: str = "N/A"
    identifier_one: str = "N/A"
    latitude: float = 0.0
    longitude: float = 0.0
    altitude: float = 0.0
    last_sent: str = "N/A"
    last_complete: str = "N/A"
    identifier_two: str = "N/A"
    rssi: float = 0.0
    snr: float = 0.0


class LoraReader(threading.Thread):
    def __init__(
        self,
        port: str,
        window: "MainWindow",
        baudrate: int = 115200,
        callback=None,
        gps_callback=None,
    ) -> None:
        """Initialize LoRa reader thread.

        Args:
            port: Serial port identifier (e.g., 'COM3', '/dev/ttyUSB0')
            baudrate: Serial communication baud rate, defaults to 115200
            callback: Optional callback object with emit() method for data forwarding
            gps_callback: Callback to store a GPSPoint
            window: handle to window class

        Raises:
            serial.SerialException: If serial port cannot be opened or configured

        Note:
            Thread-safe initialization. Serial port is opened immediately.
            Call start() to begin reading data in separate thread.
        """

        super().__init__()
        self.window: "MainWindow" = window
        self.data_lock = threading.Lock()
        self.data: LoraDataObject = None
        self.serial_port = serial.Serial(port=port, baudrate=baudrate, timeout=1)
        self.callback = callback
        self.is_running: Bool = True

    def run(self) -> None:
        """Main thread execution loop for continuous LoRa data reception.

        Continuously monitors serial port for incoming data, parses LoRa packets,
        and forwards processed data through callback mechanism. Runs until
        stop() is called or unrecoverable error occurs.

        Raises:
            serial.SerialException: On serial port communication errors
            UnicodeDecodeError: On malformed UTF-8 data (handled gracefully)
        """

        try:
            while self.is_running:
                if self.serial_port.in_waiting > 0:
                    line = (
                        self.serial_port.readline()
                        .decode("utf-8", errors="replace")
                        .strip()
                    )
                    if line:
                        print(line)
                        with self.data_lock:
                            self.data = self.parse_lora_data(line)
                        if self.callback:
                            self.callback.emit(line)
        except serial.SerialException as e:
            print(f"Error opening Serial Port: {e}")
        except Exception as e:
            if self.callback:
                self.callback.emit(f"Error in LoRaReader: {e}")
        finally:
            if "serial_connection" in locals() and self.serial_port.is_open:
                self.serial_port.close()
                print("Serial Port closed")

    def access_lora_data(self) -> LoraDataObject:
        with self.data_lock:
            return copy.deepcopy(self.data)

    def stop(self) -> None:
        """Signal thread to stop execution gracefully.

        Sets internal flag to terminate the run() loop. Thread will complete
        current iteration and exit cleanly, closing serial resources.
        """
        self.is_running = False

    def parse_lora_data(self, line: str) -> LoraDataObject:
        """Parse raw LoRa data string into structured object.

        Validates and converts colon-separated data into LoraDataObject.
        Expected format: identifier:lat:lon:alt:last_sent:last_complete:identifier:rssi:snr

        Args:
            line: Raw LoRa data string from serial port

        Returns:
            LoraDataObject: Parsed telemetry data on successful parsing

        Raises:
            ValueError: On coordinate conversion errors (handled internally)
            IndexError: On insufficient field count (handled internally)

        Notes:
            Will always return an object. If the fields cannot be parsed,
            then we return an object with malformed flag set, and the raw_lora_string
            populated with the malformed string.
        """

        # Skip lines with any forbidden characters
        # todo:tariq Instead of just ignoring bad lines, i want to store them regardless
        #            and handle what to do with them later.
        if not re.match(r"^[\w\s:.,+-]*$", line):
            malformed = True
            # print(
            #     f"Ignored malformed line (bad characters): {line}"
            # )
            # # todo:leroy make this info more useful. Count lines and calculate % of lines that are bad?
            # self.window.statusBox.append(
            #     f"Ignored malformed line (bad characters): {line}"
            # )
            # return None
        else:
            malformed = False

        fields = line.split(":")
        if len(fields) != 9:
            raise IndexError("Raw LoRa data line should contain 9 fields.")
            return LoraDataObject(
                malformed=malformed,
                raw_lora_string=line,
            )

        else:
            try:
                identifier_one = fields[0].strip()
                latitude = self.convert_to_decimal_degrees(fields[1].strip())
                longitude = self.convert_to_decimal_degrees(fields[2].strip())
                altitude = float(fields[3].strip())
                last_sent = fields[4].strip()
                last_complete = fields[5].strip()
                identifier_two = fields[6].strip()
                rssi = float(fields[7].strip())
                snr = float(fields[8].strip())

                return LoraDataObject(
                    raw_lora_string=line,
                    identifier_one=identifier_one,
                    latitude=latitude,
                    longitude=longitude,
                    altitude=altitude,
                    last_sent=last_sent,
                    last_complete=last_complete,
                    identifier_two=identifier_two,
                    rssi=rssi,
                    snr=snr,
                )

            except (ValueError, IndexError) as e:
                print(f"Parsing error: {e} | RAW: {line}")
                self.window.statusBox.append(f"Parsing error: RAW: {line}")
                return LoraDataObject(malformed=True, raw_lora_string=line)
            except Exception as e:
                print(f"Error parsing LoraData: {e}")
                self.window.statusBox.append(f"Error parsing LoraData: {e}")
                return LoraDataObject(malformed=True, raw_lora_string=line)

    @staticmethod
    def convert_to_decimal_degrees(coordinate_string: str) -> float:
        """Convert coordinate string to decimal degrees format.

        Handles multiple coordinate formats including directional suffixes
        (N/S/E/W) and signed decimal degrees. Applies proper sign conversion
        for southern latitudes and western longitudes.

        Args:
            coordinate_string: Coordinate with optional direction suffix
                              (e.g., "45.123N", "-122.456", "123.789W")

        Returns:
            Coordinate value in decimal degrees (negative for S/W directions)

        Raises:
            ValueError: If coordinate_string cannot be converted to float
            IndexError: If coordinate_string is empty or malformed
        """

        if coordinate_string[-1] in ["N", "S", "E", "W"]:
            direction = coordinate_string[-1]
            coordinate = float(coordinate_string[:-1])
            if direction in ["S", "W"]:
                coordinate *= -1
            return coordinate
        else:
            return float(coordinate_string)


class LoRaCommandSender:
    def __init__(self, serial_port):
        self.serial_port = serial_port

    def send_command(self, command: str) -> None:
        valid_commands = ["IDLE", "CUT", "OPEN", "CLOSE"]
        try:
            if command in valid_commands:
                self.serial_port.write(command.encode("ascii"))
            else:
                print(f"Invalid command: {command}")
        except Exception as err:
            print(f"Error sending commands: {err}")
