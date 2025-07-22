"""
Mock Serial Connection Testing for LoRa Ground Station

This demonstrates how to create fake serial ports for testing hardware-dependent code.
We'll mock the serial.Serial interface used by LoraReader and GroundStationArduino.
"""

import pytest
from unittest.mock import Mock, patch, MagicMock
import threading
import time
import queue
from io import BytesIO


class MockSerial:
    """A complete fake serial port that behaves like serial.Serial
    
    This mock implements the exact interface your code expects from serial.Serial.
    It allows you to control what data is "received" and verify what was "sent".
    """
    
    def __init__(self, port=None, baudrate=9600, timeout=1):
        """Initialize mock serial port with same signature as real serial.Serial"""
        self.port = port
        self.baudrate = baudrate  
        self.timeout = timeout
        self.is_open = True
        
        # Queues to simulate data flow
        self._incoming_data = queue.Queue()  # Data to be "read" by your code
        self._outgoing_data = []             # Data "written" by your code
        
        # State tracking
        self._bytes_waiting = 0
        self._read_position = 0
        
    def write(self, data):
        """Mock the write() method - records what your code tries to send"""
        if isinstance(data, str):
            data = data.encode('utf-8')
        self._outgoing_data.append(data)
        return len(data)
    
    def readline(self):
        """Mock the readline() method - returns simulated incoming data"""
        try:
            # Try to get a line from the queue (with timeout)
            line = self._incoming_data.get(timeout=self.timeout)
            if isinstance(line, str):
                line = line.encode('utf-8')
            return line + b'\n'  # Add newline like real serial
        except queue.Empty:
            return b''  # Timeout - return empty like real serial
    
    def read(self, size=1):
        """Mock the read() method for reading specific number of bytes"""
        try:
            data = self._incoming_data.get(timeout=self.timeout)
            if isinstance(data, str):
                data = data.encode('utf-8')
            return data[:size]
        except queue.Empty:
            return b''
    
    @property 
    def in_waiting(self):
        """Mock the in_waiting property - indicates how many bytes are available"""
        return self._incoming_data.qsize()
    
    def close(self):
        """Mock the close() method"""
        self.is_open = False
    
    def __enter__(self):
        """Support context manager usage: with serial.Serial() as port:"""
        return self
        
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Support context manager usage"""
        self.close()
    
    # Test helper methods (not part of real serial interface)
    def simulate_incoming_data(self, data):
        """Test helper: Add data that your code will "receive" """
        self._incoming_data.put(data)
    
    def get_written_data(self):
        """Test helper: Get all data that your code "sent" """
        return self._outgoing_data.copy()
    
    def clear_written_data(self):
        """Test helper: Clear the sent data log"""
        self._outgoing_data.clear()


class TestMockSerialBasics:
    """Test the mock serial interface itself"""
    
    def test_mock_serial_write_and_read(self):
        """Test basic write/read functionality of our mock"""
        mock_serial = MockSerial()
        
        # Test writing data
        mock_serial.write(b"Hello Arduino")
        mock_serial.write("MOVE 180,45")  # String gets converted to bytes
        
        # Verify what was written
        written_data = mock_serial.get_written_data()
        assert len(written_data) == 2
        assert written_data[0] == b"Hello Arduino"
        assert written_data[1] == b"MOVE 180,45"
        
    def test_mock_serial_simulate_incoming_data(self):
        """Test simulating data coming FROM the Arduino TO your code"""
        mock_serial = MockSerial()
        
        # Simulate Arduino sending GPS data
        mock_serial.simulate_incoming_data("ID1:45.123:-111.456:1000:0:0:ID2:-85:8.5")
        
        # Your code would call readline() and get this data
        received = mock_serial.readline()
        assert b"ID1:45.123:-111.456:1000:0:0:ID2:-85:8.5" in received
        
    def test_mock_serial_in_waiting_property(self):
        """Test the in_waiting property that indicates available data"""
        mock_serial = MockSerial()
        
        # Initially no data waiting
        assert mock_serial.in_waiting == 0
        
        # Add some data
        mock_serial.simulate_incoming_data("line 1")
        mock_serial.simulate_incoming_data("line 2")
        
        # Should show 2 items waiting
        assert mock_serial.in_waiting == 2
        
        # Read one line
        mock_serial.readline()
        
        # Should show 1 item waiting
        assert mock_serial.in_waiting == 1


class TestLoraReaderWithMockSerial:
    """Test your actual LoraReader class using mock serial ports"""
    
    @patch('lora_reader.serial.Serial')
    def test_lora_reader_initialization(self, mock_serial_class):
        """Test LoraReader initialization with mock serial"""
        # Set up the mock to return our MockSerial instance
        mock_port = MockSerial()
        mock_serial_class.return_value = mock_port
        
        # Import here to avoid circular import issues
        from lora_reader import LoraReader
        
        # Create LoraReader - should use our mock instead of real serial
        reader = LoraReader(port="/dev/fake", window=Mock())
        
        # Verify serial.Serial was called with correct parameters
        mock_serial_class.assert_called_once_with(
            port="/dev/fake", 
            baudrate=115200, 
            timeout=1
        )
        
        # Verify reader was created successfully
        assert reader.serial_port is mock_port
        
    @patch('lora_reader.serial.Serial')
    def test_lora_reader_parses_valid_data(self, mock_serial_class):
        """Test LoraReader parsing valid LoRa telemetry data"""
        mock_port = MockSerial()
        mock_serial_class.return_value = mock_port
        
        from lora_reader import LoraReader
        
        # Create reader with mock callback
        mock_callback = Mock()
        reader = LoraReader(port="/dev/fake", window=Mock(), callback=mock_callback)
        
        # Simulate valid LoRa data coming in
        valid_lora_packet = "BALLOON1:45.123:-111.456:1500:12:10:BALLOON1:-75:12.5"
        mock_port.simulate_incoming_data(valid_lora_packet)
        
        # Run the reader briefly
        reader_thread = threading.Thread(target=reader.run)
        reader_thread.start()
        
        # Let it process the data
        time.sleep(0.1)
        reader.stop()
        reader_thread.join(timeout=1)
        
        # Verify the data was parsed correctly  
        parsed_data = reader.access_lora_data()
        assert parsed_data is not None
        assert parsed_data.latitude == 45.123
        assert parsed_data.longitude == -111.456
        assert parsed_data.altitude == 1500.0
        
        # Verify callback was called
        mock_callback.emit.assert_called()


class TestGroundStationArduinoWithMockSerial:
    """Test your GroundStationArduino class using mock serial ports"""
    
    @patch('ground_station_arduino.serial.Serial')
    def test_arduino_move_position_command(self, mock_serial_class):
        """Test that move_position sends correct commands to Arduino"""
        mock_port = MockSerial()
        mock_serial_class.return_value = mock_port
        
        from ground_station_arduino import GroundStationArduino
        
        # Create Arduino controller with mock serial
        arduino = GroundStationArduino(com_port="/dev/fake", baudrate=9600)
        
        # Send a move command
        arduino.move_position(azimuth=180.0, elevation=45.0)
        
        # Verify the correct command was sent
        written_data = mock_port.get_written_data()
        assert len(written_data) == 1
        assert written_data[0] == b"M180.0,45.0"
        
    @patch('ground_station_arduino.serial.Serial')  
    def test_arduino_emergency_stop(self, mock_serial_class):
        """Test emergency stop command"""
        mock_port = MockSerial()
        mock_serial_class.return_value = mock_port
        
        from ground_station_arduino import GroundStationArduino
        
        arduino = GroundStationArduino(com_port="/dev/fake", baudrate=9600)
        
        # Send emergency stop
        arduino.send_emergency_stop()
        
        # Verify emergency stop command was sent
        written_data = mock_port.get_written_data() 
        assert len(written_data) == 1
        assert written_data[0] == b"E"
        
    @patch('ground_station_arduino.serial.Serial')
    def test_arduino_gps_request_response(self, mock_serial_class):
        """Test requesting GPS coordinates from Arduino"""
        mock_port = MockSerial()
        mock_serial_class.return_value = mock_port
        
        from ground_station_arduino import GroundStationArduino
        
        arduino = GroundStationArduino(com_port="/dev/fake", baudrate=9600)
        
        # Simulate Arduino responding with GPS coordinates
        # Format appears to be: "lat,lon,alt" with direction indicators
        mock_gps_response = b"45.123N,-111.456W,1500\r\n"
        mock_port.simulate_incoming_data(mock_gps_response.decode())
        
        # Request GPS location
        coordinates = arduino.request_gps_location()
        
        # Verify GPS request command was sent
        written_data = mock_port.get_written_data()
        assert b"G" in written_data  # GPS request command
        
        # Note: The actual parsing logic might need adjustment based on your Arduino firmware


class TestSerialCommunicationErrorHandling:
    """Test how your code handles serial communication errors"""
    
    @patch('lora_reader.serial.Serial')
    def test_lora_reader_handles_serial_exception(self, mock_serial_class):
        """Test LoraReader handles serial port errors gracefully"""
        # Configure mock to raise SerialException
        import serial
        mock_serial_class.side_effect = serial.SerialException("Port not found")
        
        from lora_reader import LoraReader
        
        # This should not crash, even if serial port fails
        try:
            reader = LoraReader(port="/dev/nonexistent", window=Mock())
            # If we get here, the exception was handled
        except serial.SerialException:
            pytest.fail("LoraReader should handle SerialException gracefully")
    
    def test_mock_serial_timeout_behavior(self):
        """Test mock serial timeout behavior matches real serial"""
        mock_serial = MockSerial(timeout=0.1)  # Very short timeout
        
        # Try to read when no data is available
        start_time = time.time()
        result = mock_serial.readline()
        elapsed = time.time() - start_time
        
        # Should return empty bytes after timeout
        assert result == b''
        assert elapsed >= 0.1  # Should have waited at least the timeout period


if __name__ == "__main__":
    # Run the tests
    pytest.main([__file__, "-v"])