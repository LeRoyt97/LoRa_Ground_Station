import dataclasses
from typing import List, Optional, Union, Dict
from dataclasses import field
from lora_reader import LoraDataObject

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

    Contains all telemetry data, reception metadata, and validation status
    for comprehensive flight analysis and debugging. Used as database
    storage format to preserve complete packet history.

    Data Flow Architecture:
        Raw LoRa → LoraDataObject → PacketRecord (DB storage)
                                 ↘
                                  GPSPoint (real-time tracking)

    Attributes:
        receive_timestamp: When ground station received packet (ISO format string)
        lora_data: Complete parsed telemetry from balloon
        raw_packet: Original packet string for debugging
        packet_sequence: Expected sequence number (if available)
        signal_quality: Reception quality metrics dictionary
        validation_errors: List of validation issues found
        ground_station_location: Receiving station coordinates (lat, lon)
    """

    receive_timestamp: str
    lora_data: LoraDataObject
    raw_packet: str
    packet_sequence: Optional[int] = None
    signal_quality: Optional[Dict[str, float]] = None  # {'rssi': -85, 'snr': 8.5}
    validation_errors: List[str] = field(default_factory=list)
    ground_station_location: Optional[tuple] = None  # (lat, lon)


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

    timestamp: str
    lora_data: LoraDataObject
    is_valid: bool
    velocity: Optional[float] = None
    distance_from_previous: Optional[float] = None


class FlightTracker:
    """GPS track storage and management for balloon flight operations.

    Provides two-tier storage system for real-time GPS tracking with in-memory
    session storage and persistent database backup. Handles data validation,
    flight session management, and track export functionality.
    """

    def __init__(self) -> None:
        """Initialize flight tracker with empty session and database connection.

        Creates new flight session storage and establishes connection to
        persistent SQLite database for historical flight data.
        """
        pass

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
            Automatically triggers database backup every 100 points to prevent
            data loss during extended flight operations. Invalid GPS fixes are
            stored but flagged for filtering during map display.
        """
        pass

    def get_recent_track(self, n: int = 50) -> List[GPSPoint]:
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

        Note:
            Only returns validated GPS points suitable for map plotting.
            Invalid GPS fixes are automatically filtered out.
        """
        pass

    def get_full_track(self, include_invalid: bool = False) -> List[GPSPoint]:
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
        pass

    def start_new_flight(self, flight_notes: str = "") -> bool:
        """Begin new flight session and archive current track data.

        Saves current flight track to persistent database, resets session
        storage, and initializes new flight with optional metadata.
        Critical for maintaining flight session boundaries.

        Args:
            flight_notes: Optional description or metadata for new flight

        Returns:
            True if new flight session started successfully,
            False if database save operation failed

        Raises:
            DatabaseError: If current flight cannot be saved to database
            ValueError: If flight_notes contain invalid characters

        Note:
            Current flight data is permanently saved before reset.
            Ensure all analysis is complete before calling this method.
        """
        pass

    def export_track(self, format: str = "gpx", filename: Optional[str] = None) -> str:
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

        Note:
            GPX format is recommended for compatibility with most GPS software.
            Only exports validated GPS points unless format supports metadata.
        """
        pass

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
        pass

    def _backup_to_database(self) -> bool:
        """Perform incremental backup of current session to persistent storage.

        Saves recent GPS points to SQLite database for crash recovery
        without disrupting current flight operations. Called automatically
        during normal operation.

        Returns:
            True if backup completed successfully,
            False if database operation failed

        Note:
            Private method for crash recovery. Does not affect current
            session data or flight operations if backup fails.
        """
        pass
