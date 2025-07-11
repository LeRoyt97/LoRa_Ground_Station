import dataclasses
import re
import serial
import threading

@dataclasses.dataclass
class LoraDataObject:
    identifier_one: str
    latitude: float
    longitude: float
    altitude: float
    last_sent: str
    last_complete: str
    identifier_two: str


class LoraReader(threading.Thread):
    def __init__(self, port, baudrate=115200, callback=None):
        super().__init__()
        self.data = None
        self.serial_port = serial.Serial(port=port, baudrate=baudrate, timeout=1)
        self.callback = callback
        self.is_running = True


    def run(self):
        try:
            while self.is_running:
                if self.serial_port.in_waiting > 0:
                    print("Debug: LoRa Run if")
                    # TODO:LeRoy: Check that error handling works
                    line = self.serial_port.readline().decode('utf-8', errors="replace").strip()
                    if line:
                        print(line)
                        self.data = self.parse_lora_data(line)
                        if self.callback:
                            self.callback.emit(line)
        except Exception as e:
            if self.callback:
                self.callback.emit(f"Error in LoRaReader: {e}")
        except serial.SerialException as e:
            print(f"Error opening Serial Port: {e}")
        finally:
            if 'serial_connection' in locals() and self.serial_port.is_open:
                self.serial_port.close()
                print("Serial Port closed")

    def stop(self):
        self.is_running = False


    # returns a LoraDataObject after parsing
    def parse_lora_data(self, line):

        # Skip lines with any forbidden characters
        if not re.match(r'^[\w\s:.,+-]*$', line):
            return f"Ignored malformed line (bad characters): {line}"


        fields = line.split(':')
        if len(fields) < 7:
            return f"Not enough fields, RAW: {line}"

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
            
        except Exception as e:
            print(f"Error parsing LoraData: {e}")
        except (ValueError, IndexError) as e:
            return f"Parsing error: {e} | RAW: {line}"


    @staticmethod
    def convert_to_decimal_degrees(coordinate_string):
        # Handles both N/S/E/W directions and +/- decimal degree formats and converts str to float
        if coordinate_string[-1] in ['N', 'S', 'E', 'W']:
            direction = coordinate_string[-1]
            coordinate = float(coordinate_string[:-1])
            if direction in ['S', 'W']:
                coordinate *= -1
            return coordinate
        else:
            return float(coordinate_string)
