import dataclasses
import re
import serial
import threading


@dataclasses.dataclass
class LoraDataObject:
    """Data container for LoRa telemetry information.

    Represents a complete telemetry packet received from a LoRa transmitter,
    containing position data and transmission metadata for balloon tracking.

    Attributes:
        identifier_one: Primary identifier string for the transmitter
        latitude: Geographic latitude in decimal degrees (-90 to +90)
        longitude: Geographic longitude in decimal degrees (-180 to +180)
        altitude: Altitude above sea level in meters
        last_sent: Timestamp or sequence number of last transmission sent
        last_complete: Timestamp or sequence number of last complete transmission
        identifier_two: Secondary identifier string for validation

    Note:
        Used for safety-critical balloon tracking operations. Coordinate
        accuracy is essential for ground station pointing calculations.
    """

    identifier_one: str
    latitude: float
    longitude: float
    altitude: float
    last_sent: str
    last_complete: str
    identifier_two: str


class LoraReader(threading.Thread):
    def __init__(
        self, port: str, window, baudrate: int = 115200, callback=None
    ) -> None:
        """Initialize LoRa reader thread for telemetry reception.

        Args:
            port: Serial port identifier (e.g., 'COM3', '/dev/ttyUSB0')
            baudrate: Serial communication baud rate, defaults to 115200
            callback: Optional callback object with emit() method for data forwarding
            window: handle to window class

        Raises:
            serial.SerialException: If serial port cannot be opened or configured

        Note:
            Thread-safe initialization. Serial port is opened immediately.
            Call start() to begin reading data in separate thread.
        """
        super().__init__()
        self.window = window
        self.data = None
        self.serial_port = serial.Serial(port=port, baudrate=baudrate, timeout=1)
        self.callback = callback
        self.is_running = True

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

    def stop(self) -> None:
        """Signal thread to stop execution gracefully.

        Sets internal flag to terminate the run() loop. Thread will complete
        current iteration and exit cleanly, closing serial resources.
        """
        self.is_running = False

    def parse_lora_data(self, line: str):
        """Parse raw LoRa data string into structured telemetry object.

        Validates and converts colon-separated telemetry data into LoraDataObject.
        Expected format: identifier:lat:lon:alt:last_sent:last_complete:identifier

        Args:
            line: Raw LoRa data string from serial port

        Returns:
            LoraDataObject: Parsed telemetry data on successful parsing

        Raises:
            ValueError: On coordinate conversion errors (handled internally)
            IndexError: On insufficient field count (handled internally)

        """
        # Skip lines with any forbidden characters
        if not re.match(r"^[\w\s:.,+-]*$", line):
            print(f"Ignored malformed line (bad characters): {line}")
            self.window.statusBox.append(
                f"Ignored malformed line (bad characters): {line}"
            )
            return None

        fields = line.split(":")
        if len(fields) < 7:
            print(f"Not enough fields, RAW: {line}")
            self.window.statusBox.append(f"Not enough fields, RAW: {line}")
            return None

        try:
            identifier_one = fields[0].strip()
            raw_latitude = fields[1].strip()
            raw_longitude = fields[2].strip()
            raw_altitude = fields[3].strip()
            last_sent = fields[4].strip()
            last_complete = fields[5].strip()
            identifier_two = fields[6].strip()

            latitude = self.convert_to_decimal_degrees(raw_latitude)
            longitude = self.convert_to_decimal_degrees(raw_longitude)
            altitude = float(raw_altitude)

            return LoraDataObject(
                identifier_one=identifier_one,
                latitude=latitude,
                longitude=longitude,
                altitude=altitude,
                last_sent=last_sent,
                last_complete=last_complete,
                identifier_two=identifier_two,
            )

        except (ValueError, IndexError) as e:

            print(f"Parsing error: {e} | RAW: {line}")
            self.window.statusBox.append(f"Parsing error: RAW: {line}")
            return None
        except Exception as e:
            print(f"Error parsing LoraData: {e}")
            self.window.statusBox.append(f"Error parsing LoraData: {e}")
            return None

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
