import serial
import struct
import time
from datetime import datetime

class IMU560Parser:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        """
        Initialize IMU560 parser
        
        Args:
            port: Serial port (e.g., 'COM3' on Windows, '/dev/ttyUSB0' on Linux)
            baudrate: Communication speed (default: 115200)
        """
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        
        # Frame format constants
        self.FRAME_START = 0xFF
        self.FRAME_END = 0x03
        
    def connect(self):
        """Connect to the IMU560 device"""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
            print(f"Connected to IMU560 on {self.port} at {self.baudrate} baud")
            return True
        except Exception as e:
            print(f"Failed to connect: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from the device"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("Disconnected from IMU560")
    
    def calculate_checksum(self, data):
        """Calculate checksum for the frame"""
        return sum(data) & 0xFF
    
    def find_frame_start(self):
        """Find the start of a valid frame"""
        while True:
            byte = self.serial_conn.read(1)
            if not byte:
                return None
            if byte[0] == self.FRAME_START:
                return byte
    
    def read_frame(self):
        """Read a complete frame from the device"""
        if not self.serial_conn or not self.serial_conn.is_open:
            return None
        
        # Find frame start
        start_byte = self.find_frame_start()
        if not start_byte:
            return None
        
        # Read frame header
        header = self.serial_conn.read(4)  # VER, CMD, LEN_L, LEN_H
        if len(header) < 4:
            return None
        
        ver, cmd, len_l, len_h = header
        data_length = (len_h << 8) | len_l
        
        # Read data payload
        data = self.serial_conn.read(data_length)
        if len(data) < data_length:
            return None
        
        # Read checksum and end byte
        footer = self.serial_conn.read(2)
        if len(footer) < 2:
            return None
        
        checksum, end_byte = footer
        
        if end_byte != self.FRAME_END:
            print(f"Invalid frame end: {end_byte:02X}")
            return None
        
        # Verify checksum
        frame_data = [ver, cmd, len_l, len_h] + list(data)
        calculated_checksum = self.calculate_checksum(frame_data)
        
        if checksum != calculated_checksum:
            print(f"Checksum mismatch: {checksum:02X} vs {calculated_checksum:02X}")
            return None
        
        return {
            'version': ver,
            'command': cmd,
            'length': data_length,
            'data': data,
            'checksum': checksum
        }
    
    def parse_float32(self, data, offset):
        """Parse a 32-bit float from data"""
        return struct.unpack('<f', data[offset:offset+4])[0]
    
    def parse_float64(self, data, offset):
        """Parse a 64-bit double from data"""
        return struct.unpack('<d', data[offset:offset+8])[0]
    
    def parse_uint32(self, data, offset):
        """Parse a 32-bit unsigned integer from data"""
        return struct.unpack('<I', data[offset:offset+4])[0]
    
    def parse_imu_data(self, frame):
        """Parse IMU data from a frame based on the command type"""
        if not frame:
            return None
        
        cmd = frame['command']
        data = frame['data']
        
        result = {
            'timestamp': datetime.now(),
            'command': f"0x{cmd:02X}",
            'raw_data': data.hex()
        }
        
        try:
            # Parse based on command type
            if cmd == 0x61:  # Example: Combined sensor data
                # This is a generic parser - you may need to adjust based on actual data format
                offset = 0
                
                # Parse gyroscope data (3 x float32 = 12 bytes)
                if len(data) >= offset + 12:
                    result['gyro_x'] = self.parse_float32(data, offset)
                    result['gyro_y'] = self.parse_float32(data, offset + 4)
                    result['gyro_z'] = self.parse_float32(data, offset + 8)
                    offset += 12
                
                # Parse accelerometer data (3 x float32 = 12 bytes)
                if len(data) >= offset + 12:
                    result['accel_x'] = self.parse_float32(data, offset)
                    result['accel_y'] = self.parse_float32(data, offset + 4)
                    result['accel_z'] = self.parse_float32(data, offset + 8)
                    offset += 12
                
                # Parse magnetometer data (3 x float32 = 12 bytes)
                if len(data) >= offset + 12:
                    result['mag_x'] = self.parse_float32(data, offset)
                    result['mag_y'] = self.parse_float32(data, offset + 4)
                    result['mag_z'] = self.parse_float32(data, offset + 8)
                    offset += 12
                
                # Parse attitude data (3 x float32 = 12 bytes)
                if len(data) >= offset + 12:
                    result['roll'] = self.parse_float32(data, offset)
                    result['pitch'] = self.parse_float32(data, offset + 4)
                    result['yaw'] = self.parse_float32(data, offset + 8)
                    offset += 12
                
                # Parse position data (3 x float64 = 24 bytes)
                if len(data) >= offset + 24:
                    result['latitude'] = self.parse_float64(data, offset)
                    result['longitude'] = self.parse_float64(data, offset + 8)
                    result['altitude'] = self.parse_float64(data, offset + 16)
                    offset += 24
                
                # Parse velocity data (3 x float32 = 12 bytes)
                if len(data) >= offset + 12:
                    result['vel_north'] = self.parse_float32(data, offset)
                    result['vel_east'] = self.parse_float32(data, offset + 4)
                    result['vel_down'] = self.parse_float32(data, offset + 8)
                    offset += 12
            
            else:
                # For other command types, just store raw data
                result['data_length'] = len(data)
                result['parsed'] = False
                
        except Exception as e:
            print(f"Error parsing data: {e}")
            result['parse_error'] = str(e)
        
        return result
    
    def start_continuous_reading(self, callback=None, duration=None):
        """
        Start reading continuous data from IMU560
        
        Args:
            callback: Function to call with parsed data (optional)
            duration: How long to read in seconds (None for infinite)
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            print("Device not connected")
            return
        
        print("Starting continuous data reading...")
        print("Press Ctrl+C to stop")
        
        start_time = time.time()
        frame_count = 0
        
        try:
            while True:
                # Check duration limit
                if duration and (time.time() - start_time) > duration:
                    break
                
                # Read and parse frame
                frame = self.read_frame()
                if frame:
                    parsed_data = self.parse_imu_data(frame)
                    frame_count += 1
                    
                    if callback:
                        callback(parsed_data)
                    else:
                        self.print_parsed_data(parsed_data)
                
                time.sleep(0.001)  # Small delay to prevent CPU overload
                
        except KeyboardInterrupt:
            print(f"\nStopped reading. Processed {frame_count} frames.")
        except Exception as e:
            print(f"Error during reading: {e}")
    
    def print_parsed_data(self, data):
        """Print parsed data in a readable format"""
        if not data:
            return
        
        print(f"\n--- Frame at {data['timestamp'].strftime('%H:%M:%S.%f')[:-3]} ---")
        print(f"Command: {data['command']}")
        
        if 'gyro_x' in data:
            print(f"Gyro (rad/s): X={data['gyro_x']:.6f}, Y={data['gyro_y']:.6f}, Z={data['gyro_z']:.6f}")
        
        if 'accel_x' in data:
            print(f"Accel (m/s²): X={data['accel_x']:.6f}, Y={data['accel_y']:.6f}, Z={data['accel_z']:.6f}")
        
        if 'mag_x' in data:
            print(f"Mag (µT): X={data['mag_x']:.6f}, Y={data['mag_y']:.6f}, Z={data['mag_z']:.6f}")
        
        if 'roll' in data:
            print(f"Attitude (°): Roll={data['roll']:.3f}, Pitch={data['pitch']:.3f}, Yaw={data['yaw']:.3f}")
        
        if 'latitude' in data:
            print(f"Position: Lat={data['latitude']:.8f}°, Lon={data['longitude']:.8f}°, Alt={data['altitude']:.3f}m")
        
        if 'vel_north' in data:
            print(f"Velocity (m/s): N={data['vel_north']:.3f}, E={data['vel_east']:.3f}, D={data['vel_down']:.3f}")
        
        if 'parse_error' in data:
            print(f"Parse Error: {data['parse_error']}")
        
        if not data.get('parsed', True):
            print(f"Raw data ({data.get('data_length', 0)} bytes): {data['raw_data']}")


def main():
    """Main function to demonstrate usage"""
    # Initialize parser (adjust port as needed)
    # For Windows: port='COM3'
    # For Linux/Mac: port='/dev/ttyUSB0' or '/dev/ttyACM0'
    parser = IMU560Parser(port='/dev/ttyUSB0', baudrate=115200)
    
    # Connect to device
    if not parser.connect():
        print("Failed to connect to IMU560")
        return
    
    try:
        # Start reading continuous data for 30 seconds
        parser.start_continuous_reading(duration=30)
    
    finally:
        # Clean up
        parser.disconnect()


if __name__ == "__main__":
    main()