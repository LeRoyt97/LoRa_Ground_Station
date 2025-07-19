import dataclasses
import sqlite3
import json
import os
import re
from datetime import datetime, timezone
from typing import List, Optional, Union, Dict
from dataclasses import field
from lora_reader import LoraDataObject

BACKUP_INTERVAL = 100
ROUGH_METERS_PER_DEGREE = 40075.0 / 360 # circumference ~= 40075 km

"""
-- Flight sessions
CREATE TABLE flights (
    flight_id INTEGER PRIMARY KEY,
    start_time TIMESTAMP,
    end_time TIMESTAMP,
    notes TEXT,
    ground_station_lat REAL,
    ground_station_lon REAL
);

-- Complete packet records
CREATE TABLE packet_records (
    packet_id INTEGER PRIMARY KEY,
    flight_id INTEGER,
    receive_timestamp TIMESTAMP,
    raw_packet TEXT,
    latitude REAL,
    longitude REAL,
    altitude REAL,
    rssi REAL,
    snr REAL,
    validation_errors TEXT,  -- JSON string
    FOREIGN KEY (flight_id) REFERENCES flights(flight_id)
);

-- Index for time-based queries
CREATE INDEX idx_packet_timestamp ON packet_records(receive_timestamp);
"""


@dataclasses.dataclass
class PacketRecord:
    """Complete LoRa packet record for persistent storage.

    Wraps GPSPoint with additional metadata for database storage and analysis.
    Contains reception metadata and validation status for comprehensive flight
    analysis and debugging.

    Data Flow Architecture:
        Raw LoRa → LoraDataObject → GPSPoint (real-time tracking)
                                      ↓
                                  PacketRecord (DB storage)

    Attributes:
        gps_point: Core GPS telemetry data with computed metrics
        signal_quality: Reception quality metrics dictionary
        validation_errors: List of validation issues found
        ground_station_location: Ground station coordinates when packet received
    """

    gps_point: GPSPoint
    signal_quality: Optional[Dict[str, float]] = None  # {'rssi': -85, 'snr': 8.5}
    validation_errors: List[str] = field(default_factory=list)
    ground_station_location: Optional[dict] = None  # {'lat': float, 'lon': float}


@dataclasses.dataclass
class GPSPoint:
    """Container for timestamped GPS telemetry data.

    Represents a single GPS measurement from balloon telemetry with validation
    status and computed metrics for flight path analysis.

    Attributes:
        timestamp: UTC timestamp when GPS point was received (ISO format string)
        lora_data: Complete LoRa telemetry packet containing GPS coordinates
        is_valid: Whether GPS coordinates passed validation checks
        velocity: Computed ground speed in m/s from previous point (optional)
        distance_from_previous: Distance in meters from last valid point (optional)
    """

    timestamp: str = "N/A"
    lora_data: LoraDataObject
    is_valid: bool = True
    velocity: Optional[float] = None
    distance_from_previous: Optional[float] = None


class FlightTracker:
    """GPS track storage and management for balloon flight operations.

    Provides two-tier storage system for real-time GPS tracking with in-memory
    session storage and persistent database backup. Handles data validation,
    flight session management, and track export functionality.
    """

    def __init__(
        self, 
        ground_station_coords: dict, 
        status_box_callback: Optional[Callable[[str], None]] = None, 
        db_path: str = "flight_data.db"
    ) -> None:
        """Initialize flight tracker with empty session and database connection.

        Creates new flight session storage and establishes connection to
        persistent SQLite database for historical flight data.

        Args:
            ground_station_coords: Ground station coordinates dict with 'lat' and 'lon' keys (required)
            db_path: Path to SQLite database file, defaults to "flight_data.db"
        """
        # Validate ground station coordinates
        if not isinstance(ground_station_coords, dict):
            raise TypeError("ground_station_coords must be a dictionary")
        if 'lat' not in ground_station_coords or 'lon' not in ground_station_coords:
            raise ValueError("ground_station_coords must contain 'lat' and 'lon' keys")
        
        self.ground_station_coords = ground_station_coords
        self.status_callback = status_box_callback
        self.db_path = db_path
        self.points_till_backup = BACKUP_INTERVAL
        self.db = self._initialize_database()
        self.current_flight_id: Optional[int] = None
        self.current_session_points: List[GPSPoint] = []

    def add_gps_point(self, lora_data: LoraDataObject) -> bool:
        """Add new GPS telemetry point to current flight track.

        Validates incoming GPS data, creates timestamped track point with current
        ISO timestamp, and appends to current flight session. Performs basic
        validation checks and optionally triggers periodic database backup.

        Args:
            lora_data: Parsed LoRa telemetry containing GPS coordinates, altitude,
                      and signal quality metrics from balloon transmitter

        Returns:
            True if GPS point was successfully added and validated,
            False if point was rejected due to validation failures

        Raises:
            ValueError: If lora_data contains invalid coordinate values
            TypeError: If lora_data is not a LoraDataObject instance

        Note:
            Automatically triggers database backup every {BACKUP_INTERVAL} points to prevent
            data loss during extended flight operations. Invalid GPS fixes are
            stored but flagged for filtering during map display.
        """
        if not isinstance(lora_data, LoraDataObject):
            raise TypeError("lora_data must be a LoraDataObject instance")

        # Create timestamp for this GPS point
        timestamp = datetime.now(timezone.utc).isoformat()

        # Validate GPS coordinates
        is_valid = self._validate_gps_point(lora_data)

        if not is_valid:
            raise ValueError(f"The passed LoraDataObject has invalid GPS coordinates.\nLon: {lora_data.longitude}, Lat: {lora_data.latitude}")

        # Calculate velocity and distance if we have previous points
        velocity = None
        distance_from_previous = None
        if self.current_session_points:
            last_valid_point = None
            for point in reversed(self.current_session_points):
                if point.is_valid:
                    last_valid_point = point
                    break

            if last_valid_point:
                # Simple velocity calculation 
                # todo:tariq enhance with proper geodesic math
                # todo:tariq like the haversine formula
                time_diff = ( 
                    datetime.fromisoformat(timestamp) 
                    - datetime.fromisoformat(last_valid_point.timestamp)
                ).total_seconds()
                if time_diff > 0:
                    lat_diff = lora_data.latitude - last_valid_point.lora_data.latitude
                    lon_diff = lora_data.longitude - last_valid_point.lora_data.longitude
                    distance_degrees_from_previous = (
                        (lat_diff**2 + lon_diff**2) ** 0.5
                    ) * ROUGH_METERS_PER_DEGREE  # Rough conversion to meters
                    velocity = distance_from_previous / time_diff

        # Create GPS point
        gps_point = GPSPoint(
            timestamp=timestamp,
            lora_data=lora_data,
            is_valid=is_valid,
            velocity=velocity,
            distance_from_previous=distance_from_previous,
        )

        # Add to current session
        self.current_session_points.append(gps_point)

        # Periodic database backup
        # Backup every {BACKUP_INTERVAL} new points
        self.points_till_backup -= 1
        if self.points_till_backup == 0:
            self._backup_to_database()
            self.points_till_backup = BACKUP_INTERVAL

        return is_valid

    def get_recent_valid_points(self, n: int = 50) -> List[GPSPoint]:
        """Retrieve most recent GPS points for real-time map display.

        Returns the last N GPS points from current flight session, filtered
        to include only valid GPS coordinates. Used for updating map trail
        without processing entire flight history.

        Args:
            n: Maximum number of recent points to return, defaults to 50

        Returns:
            List of GPS points ordered chronologically (oldest first).
            Empty list if no valid points exist in current session.

        Raises:
            ValueError: If n is negative or exceeds reasonable limits
        """
        if n < 0:
            raise ValueError("n must be non-negative")

        # Filter for valid points only
        valid_points = [
            point for point in self.current_session_points if point.is_valid
        ]

        # Return last n points
        return valid_points[-n:] if len(valid_points) > n else valid_points

    def get_full_history(self, include_invalid: bool = False) -> List[GPSPoint]:
        """Retrieve complete GPS track for current flight session.

        Returns all GPS points from flight start to present, optionally
        including invalid GPS fixes for debugging and analysis purposes.
        Used for track export and post-flight analysis.

        Args:
            include_invalid: Whether to include GPS points that failed validation

        Returns:
            Complete chronological list of GPS points from current flight.
            Empty list if no flight session is active.

        Note:
            Large flights may return thousands of points. Consider using
            get_recent_track() for real-time display updates.
        """
        if include_invalid:
            return self.current_session_points.copy()
        else:
            return [point for point in self.current_session_points if point.is_valid]

    def start_new_flight(self, flight_notes: str = "N/A", store_old_invalid: bool = False) -> bool:
        """Begin new flight session and archive current track data.

        Saves current flight track to persistent database, resets session
        storage, and initializes new flight with optional metadata.
        Critical for maintaining flight session boundaries.

        Args:
            flight_notes: Optional description or metadata for new flight
            store_old_invalid: Optional bool

        Returns:
            True if new flight session started successfully,
            False if database save operation failed

        Raises:
            DatabaseError: If current flight cannot be saved to database
            ValueError: If flight_notes contain invalid characters

        Note:
            Current flight data is permanently saved before reset.
            Ensure all analysis is complete before calling this method.
            By default does not store invalid entries.
        """
        if not re.match(r"^[\w -]*$", flight_notes):
            raise ValueError(
                "flight_notes may only contain alphanumerics, spaces, hyphens, and underscores"
            )

        try:
            cursor = self.db.cursor()
            # check if we need to store previous flight data
            if self.current_session_points and self.current_flight_id:
                # Store the old session data into database
                points = self.get_full_history(include_invalid=store_old_invalid)
                for point in points:
                    validation_errors = []
                    if point.lora_data.malformed: 
                        validation_errors.append("Invalid LoRa String.")
                    elif not self._validate_gps_point(point.lora_data):
                        validation_errors.append("Invalid GPS information.")
                    cursor.execute(
                        """
                        INSERT OR REPLACE INTO packet_records 
                        (flight_id, receive_timestamp, raw_packet, latitude, longitude, 
                         altitude, rssi, snr, validation_errors)
                        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
                        """,
                        (
                            self.current_flight_id,
                            point.timestamp,
                            point.lora_data.raw_lora_string,
                            point.lora_data.latitude,
                            point.lora_data.longitude,
                            point.lora_data.altitude,
                            point.lora_data.rssi,
                            point.lora_data.snr,
                            json.dumps(validation_errors)
                        ),
                    )
                # update the end time of the current flight
                cursor.execute(
                    """
                    UPDATE flights
                    SET end_time = ?
                    WHERE flight_id = ?
                    """,
                    (
                        datetime.now(timezone.utc).isoformat(),
                        self.current_flight_id
                    )
                )
            # if not currently in a flight
            # create new row in flight
            cursor.execute(
                """
                INSERT INTO flights (start_time, notes, ground_station_lat, ground_station_lon)
                VALUES (?, ?, ?, ?)
                """,
                (
                    datetime.now(timezone.utc).isoformat(),
                    flight_notes,
                    self.ground_station_coords['lat'],
                    self.ground_station_coords['lon']
                )
            )
            self.current_session_points = []
            self.current_flight_id = cursor.lastrowid
            self.db.commit()
            return True
        except sqlite3.Error as err:
            error_msg = f"Database error during flight start: {err}"
            if self.status_callback:
                self.status_callback(error_msg)
            print(error_msg)
            return False

    def export_track(
            self, format: str = "gpx", filename: str = datetime.now(timezone.utc).isoformat()
    ) -> str:
        """Export current flight track to standard GPS file format.

        Converts current flight session to industry-standard GPS formats
        for analysis in external mapping software and flight planning tools.
        Automatically generates filename if not provided.

        Args:
            format: Export format ('gpx', 'kml', 'csv'), defaults to 'gpx'
            filename: Optional output filename, auto-generated if None

        Returns:
            Full path to exported file

        Raises:
            ValueError: If format is not supported
            IOError: If file cannot be written to specified location
            RuntimeError: If no GPS data exists to export

        Format Specifications:
            - GPX: GPS Exchange Format (https://www.topografix.com/gpx.asp)
            - KML: Keyhole Markup Language (https://developers.google.com/kml/documentation/kmlreference)  
            - CSV: Comma-Separated Values (https://tools.ietf.org/html/rfc4180)

        Note:
            GPX format is recommended for compatibility with most GPS software.
            Only exports validated GPS points. Invalid points are excluded.
        """
        formats = ["gpx", "kml", "csv"]
        try:
            match format:
                case "gpx":
                    return self._export_gpx(filename)
                case "kml":
                    return self._export_kml(filename)
                case "csv":
                    return self._export_csv(filename)
                case _:
                    raise ValueError(
                        f"Unsupported format '{format}'. Supported formats: {formats}."
                    )
        except IOError as err:
            error_msg = f"Failed to write GPS export file: {err}"
            if self.status_callback:
                self.status_callback(error_msg)
            raise IOError(error_msg)
        except RuntimeError as err:
            error_msg = f"Export failed: {str(err)}"
            if self.status_callback:
                self.status_callback(error_msg)
            raise



    def _export_gpx(self, filename: str) -> str:
        """Export GPS track to GPX format.
        
        GPX (GPS Exchange Format) is an XML schema for GPS data exchange.
        Specification: https://www.topografix.com/gpx.asp
        
        Args:
            filename: Output filename (will add .gpx extension if missing)
            
        Returns:
            Full path to exported GPX file
            
        Raises:
            RuntimeError: If no valid GPS data exists to export
            IOError: If error during file opening / writing process
            ValueError: Must be UTF-8
        """
        valid_points = self.get_full_history(include_invalid=False)
        if not valid_points:
            raise RuntimeError("No valid GPS data exists to export")
            
        # Ensure .gpx extension
        if not filename.endswith('.gpx'):
            filename += '.gpx'
            
        # GPX XML structure
        gpx_content = '''<?xml version="1.0" encoding="UTF-8"?>
<gpx version="1.1" creator="LoRa Ground Station" xmlns="http://www.topografix.com/GPX/1/1">
  <trk>
    <name>Balloon Flight Track</name>
    <trkseg>
'''
        
        for point in valid_points:
            gpx_content += f'''      <trkpt lat="{point.lora_data.latitude}" lon="{point.lora_data.longitude}">
        <ele>{point.lora_data.altitude}</ele>
        <time>{point.timestamp}</time>
      </trkpt>
'''
        
        gpx_content += '''    </trkseg>
  </trk>
</gpx>'''
        
        try:
            with open(filename, 'w', encoding='utf-8') as f:
                f.write(gpx_content)
        except (OSError, UnicodeEncodeError) as err:
            error_msg = f"Cannot write GPX file {filename}: {err}"
            if self.status_callback:
                self.status_callback(error_msg)
            raise IOError(error_msg)
        except Exception as err:
            error_msg = f"Unexpected error writing GPX file {filename}: {err}"
            if self.status_callback:
                self.status_callback(error_msg)
            print(error_msg)
            raise

        return os.path.abspath(filename)

    def _export_kml(self, filename: str) -> str:
        """Export GPS track to KML format.
        
        KML (Keyhole Markup Language) is Google Earth's XML format.
        Specification: https://developers.google.com/kml/documentation/kmlreference
        
        Args:
            filename: Output filename (will add .kml extension if missing)
            
        Returns:
            Full path to exported KML file
            
        Raises:
            RuntimeError: If no valid GPS data exists to export
        """
        valid_points = self.get_full_history(include_invalid=False)
        if not valid_points:
            raise RuntimeError("No valid GPS data exists to export")
            
        # Ensure .kml extension
        if not filename.endswith('.kml'):
            filename += '.kml'
            
        # KML XML structure
        kml_content = '''<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <name>Balloon Flight Track</name>
    <Placemark>
      <name>Flight Path</name>
      <LineString>
        <extrude>1</extrude>
        <tessellate>1</tessellate>
        <altitudeMode>absolute</altitudeMode>
        <coordinates>
'''
        
        # KML coordinates format: lon,lat,alt (note: lon first!)
        for point in valid_points:
            kml_content += f'          {point.lora_data.longitude},{point.lora_data.latitude},{point.lora_data.altitude}\n'
        
        kml_content += '''        </coordinates>
      </LineString>
    </Placemark>
  </Document>
</kml>'''
        
        try:
            with open(filename, 'w', encoding='utf-8') as f:
                f.write(kml_content)
        except (OSError, UnicodeEncodeError) as err:
            error_msg = f"Cannot write KML file {filename}: {err}"
            if self.status_callback:
                self.status_callback(error_msg)
            raise IOError(error_msg)
        except Exception as err:
            error_msg = f"Unexpected error writing KML file {filename}: {err}"
            if self.status_callback:
                self.status_callback(error_msg)
            print(error_msg)
            raise
            
        return os.path.abspath(filename)

    def _export_csv(self, filename: str) -> str:
        """Export GPS track to CSV format.
        
        CSV format with headers for spreadsheet analysis.
        RFC 4180 compliant: https://tools.ietf.org/html/rfc4180
        
        Args:
            filename: Output filename (will add .csv extension if missing)
            
        Returns:
            Full path to exported CSV file
            
        Raises:
            RuntimeError: If no valid GPS data exists to export
        """
        valid_points = self.get_full_history(include_invalid=False)
        if not valid_points:
            raise RuntimeError("No valid GPS data exists to export")
            
        # Ensure .csv extension
        if not filename.endswith('.csv'):
            filename += '.csv'
            
        # CSV headers and data
        csv_content = "timestamp,latitude,longitude,altitude,rssi,snr,velocity,distance_from_previous\n"
        
        for point in valid_points:
            csv_content += f'"{point.timestamp}",{point.lora_data.latitude},{point.lora_data.longitude},{point.lora_data.altitude},{point.lora_data.rssi},{point.lora_data.snr},{point.velocity or ""},{point.distance_from_previous or ""}\n'
        
        try:
            with open(filename, 'w', encoding='utf-8') as f:
                f.write(csv_content)
        except (OSError, UnicodeEncodeError) as err:
            error_msg = f"Cannot write CSV file {filename}: {err}"
            if self.status_callback:
                self.status_callback(error_msg)
            raise IOError(error_msg)
        except Exception as err:
            error_msg = f"Unexpected error writing CSV file {filename}: {err}"
            if self.status_callback:
                self.status_callback(error_msg)
            print(error_msg)
            raise
            
        return os.path.abspath(filename)

    def _validate_gps_point(self, lora_data: LoraDataObject) -> bool:
        """Validate GPS coordinates and signal quality metrics.

        Performs bounds checking, signal quality assessment, and velocity
        filtering to identify invalid GPS fixes that should not be used
        for navigation or mapping display.

        Args:
            lora_data: LoRa telemetry data containing GPS coordinates

        Returns:
            True if GPS data passes all validation checks,
            False if data appears invalid or corrupted

        Note:
            Private method used internally by add_gps_point().
            Validation criteria include coordinate bounds, signal strength,
            and velocity constraints based on balloon flight characteristics.
        """

        # Check coordinate bounds
        if not (-90 <= lora_data.latitude <= 90):
            return False
        if not (-180 <= lora_data.longitude <= 180):
            return False

        # Check altitude bounds (reasonable for balloon flights)
        if not (0 <= lora_data.altitude <= 50000):  # 0 to 50km altitude
            return False

        # Check signal quality
        if lora_data.rssi < -120:  # Very weak signal threshold
            return False

        return True

    def _backup_to_database(self, include_invalid: bool = True) -> bool:
        """Perform incremental backup of current session to persistent storage.

        Saves recent GPS points to SQLite database for crash recovery
        without disrupting current flight operations. Called automatically
        during normal operation.

        Args:
            include_invalid: Whether to backup invalid GPS points, defaults to True

        Returns:
            True if backup completed successfully,
            False if database operation failed

        Note:
            Private method for crash recovery. Does not affect current
            session data or flight operations if backup fails.
        """

        try:
            # todo:tariq i have this current_flight_id that isnt getting set anywhere
            if not self.current_session_points or not self.current_flight_id:
                return True  # Nothing to backup

            cursor = self.db.cursor()

            # Get points that haven't been backed up yet
            points_to_backup = self.current_session_points[-BACKUP_INTERVAL:]

            for point in points_to_backup:
                # Skip invalid points if include_invalid is False
                if not include_invalid and not point.is_valid:
                    continue
                    
                # Create PacketRecord for database storage
                packet_record = PacketRecord(
                    gps_point=point,
                    signal_quality={"rssi": point.lora_data.rssi, "snr": point.lora_data.snr},
                    validation_errors=(
                        [] if point.is_valid else ["GPS validation failed"]
                    ),
                    ground_station_location=self.ground_station_coords,
                )

                # Insert into database
                cursor.execute(
                    """
                    INSERT OR REPLACE INTO packet_records 
                    (flight_id, receive_timestamp, raw_packet, latitude, longitude, 
                     altitude, rssi, snr, validation_errors)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                    (
                        self.current_flight_id,
                        packet_record.gps_point.timestamp,
                        packet_record.gps_point.lora_data.raw_lora_string,
                        packet_record.gps_point.lora_data.latitude,
                        packet_record.gps_point.lora_data.longitude,
                        packet_record.gps_point.lora_data.altitude,
                        packet_record.gps_point.lora_data.rssi,
                        packet_record.gps_point.lora_data.snr,
                        json.dumps(packet_record.validation_errors),
                    ),
                )

            self.db.commit()
            return True

        except sqlite3.Error as e:
            print(f"Database backup error: {e}")
            return False

    def _initialize_database(self) -> sqlite3.Connection:
        """Initialize SQLite database with schema if it doesn't exist.

        Creates database file and tables according to documented schema.
        Sets up proper indexes for performance.

        Returns:
            sqlite3.Connection: Database connection ready for use

        Raises:
            sqlite3.Error: If database cannot be created or accessed
        """
        try:
            # Create database connection
            conn = sqlite3.connect(self.db_path)
            conn.execute("PRAGMA foreign_keys = ON")  # Enable foreign key constraints

            cursor = conn.cursor()

            # Create flights table
            cursor.execute(
                """
                CREATE TABLE IF NOT EXISTS flights (
                    flight_id INTEGER PRIMARY KEY,
                    start_time TIMESTAMP,
                    end_time TIMESTAMP,
                    notes TEXT,
                    ground_station_lat REAL,
                    ground_station_lon REAL
                )
            """
            )

            # Create packet_records table
            cursor.execute(
                """
                CREATE TABLE IF NOT EXISTS packet_records (
                    packet_id INTEGER PRIMARY KEY,
                    flight_id INTEGER,
                    receive_timestamp TIMESTAMP,
                    raw_packet TEXT,
                    latitude REAL,
                    longitude REAL,
                    altitude REAL,
                    rssi REAL,
                    snr REAL,
                    validation_errors TEXT,
                    FOREIGN KEY (flight_id) REFERENCES flights(flight_id)
                )
            """
            )

            # Create index for time-based queries
            cursor.execute(
                """
                CREATE INDEX IF NOT EXISTS idx_packet_timestamp 
                ON packet_records(receive_timestamp)
            """
            )

            conn.commit()
            return conn

        except sqlite3.Error as e:
            print(f"Database initialization error: {e}")
            raise

    def close_database(self) -> None:
        """Close database connection gracefully.

        Should be called when shutting down the application to ensure
        proper database cleanup.
        """
        if self.db:
            self.db.close()
