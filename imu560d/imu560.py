#!/usr/bin/env python3
"""
IMU560 Combined Inertial Navigation System Controller
Compatible with Python 3.11.2
"""

import serial
import struct
import json
import logging
import time
from typing import Dict, List, Optional, Tuple, Union
import threading
from dataclasses import dataclass
from enum import IntEnum

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class IMUError(Exception):
    """Custom exception for IMU errors"""
    pass

class ErrorCode(IntEnum):
    """IMU Error Codes"""
    NO_ERROR = 0x00
    ERROR = 0x01
    INVALID_FRAME = 0x04
    INVALID_PARAMETER = 0x09
    NOT_READY = 0x0A

class Commands(IntEnum):
    """IMU Command Codes"""
    # Acknowledgment
    IMU_ACK = 0x01
    
    # Settings
    IMU_SAVE_SETTINGS = 0x24
    
    # Output configuration
    IMU_SET_DEFAULT_OUTPUT_MASK = 0x50
    IMU_GET_DEFAULT_OUTPUT_MASK = 0x51
    IMU_RET_DEFAULT_OUTPUT_MASK = 0x52
    
    # Continuous mode
    IMU_SET_CONTINUOUS_MODE = 0x53
    IMU_GET_CONTINUOUS_MODE = 0x54
    IMU_RET_CONTINUOUS_MODE = 0x55
    
    # Data output
    IMU_GET_DEFAULT_OUTPUT = 0x56
    IMU_RET_DEFAULT_OUTPUT = 0x57
    IMU_GET_SPECIFIC_OUTPUT = 0x58
    IMU_RET_SPECIFIC_OUTPUT = 0x59
    
    # Protocol settings
    IMU_SET_PROTOCOL_MODE = 0x12
    IMU_GET_PROTOCOL_MODE = 0x13
    IMU_RET_PROTOCOL_MODE = 0x14
    
    # Output mode
    IMU_GET_OUTPUT_MODE = 0x16
    IMU_RET_OUTPUT_MODE = 0x17
    
    # Gravity settings
    IMU_SET_GRAVITY_MAGNITUDE = 0xB1
    IMU_GET_GRAVITY_MAGNITUDE = 0xB2
    IMU_RET_GRAVITY_MAGNITUDE = 0xB3
    
    # Magnetic calibration
    IMU_CALIB_MAG = 0x70
    IMU_CALIB_MAG_START = 0x08
    IMU_CALIB_MAG_STOP = 0x0A
    IMU_CALIB_MAG_ABANDON = 0x09
    
    # Magnetic declination
    IMU_SET_MAGNETIC_DECLINATION = 0x40
    IMU_GET_MAGNETIC_DECLINATION = 0x41
    IMU_RET_MAGNETIC_DECLINATION = 0x42
    
    # System
    IMU_SOFT_RESET = 0x88
    
    # Continuous output
    IMU_CONTINUOUS_DEFAULT_OUTPUT = 0x90

class OutputMask(IntEnum):
    """Output Mask Values"""
    QUATERNION = 0x00000001
    EULER = 0x00000002
    MATRIX = 0x00000004
    GYROSCOPES = 0x00000008
    ACCELEROMETERS = 0x00000010
    MAGNETOMETERS = 0x00000020
    TEMPERATURES = 0x00000040
    GYROSCOPES_RAW = 0x00000080
    ACCELEROMETERS_RAW = 0x00000100
    MAGNETOMETERS_RAW = 0x00000200
    TEMPERATURES_RAW = 0x00000400
    TIME_SINCE_RESET = 0x00000800
    DEVICE_STATUS = 0x00001000
    GPS_POSITION = 0x00002000
    GPS_NAVIGATION = 0x00004000
    GPS_ACCURACY = 0x00008000
    GPS_INFO = 0x00010000
    BARO_ALTITUDE = 0x00020000
    BARO_PRESSURE = 0x00040000
    POSITION = 0x00080000
    VELOCITY = 0x00100000
    ATTITUDE_ACCURACY = 0x00200000
    NAV_ACCURACY = 0x00400000
    GYRO_TEMPERATURES = 0x00800000
    GYRO_TEMPERATURES_RAW = 0x01000000
    UTC_TIME_REFERENCE = 0x02000000
    MAG_CALIB_DATA = 0x04000000
    MAG_HEADING = 0x08000000
    ODO_VELOCITY = 0x10000000
    DELTA_ANGLES = 0x20000000
    HEAVE = 0x40000000

@dataclass
class SensorData:
    """Data structure for sensor readings"""
    # Attitude (Euler angles in radians)
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    
    # Gyroscope (rad/s)
    gyro_x: float = 0.0
    gyro_y: float = 0.0
    gyro_z: float = 0.0
    
    # Accelerometer (m/s²)
    accel_x: float = 0.0
    accel_y: float = 0.0
    accel_z: float = 0.0
    
    # Magnetometer (µT)
    mag_x: float = 0.0
    mag_y: float = 0.0
    mag_z: float = 0.0
    
    # Temperature (°C)
    temp_0: float = 0.0
    temp_1: float = 0.0
    
    # Time since reset (ms)
    time_since_reset: int = 0
    
    # GPS info
    gps_itow: int = 0
    gps_flags: int = 0
    gps_num_sv: int = 0
    
    # Barometric data
    baro_altitude: int = 0  # cm
    baro_pressure: int = 0  # Pascal
    
    # Position (WGS84)
    latitude: float = 0.0   # degrees
    longitude: float = 0.0  # degrees
    altitude: float = 0.0   # meters
    
    # Velocity (m/s)
    vel_north: float = 0.0
    vel_east: float = 0.0
    vel_down: float = 0.0
    
    # UTC Time
    utc_year: int = 0
    utc_month: int = 0
    utc_day: int = 0
    utc_hour: int = 0
    utc_minute: int = 0
    utc_second: int = 0
    
    # Magnetic heading (degrees)
    mag_heading: float = 0.0

class IMU560:
    """IMU560 Combined Inertial Navigation System Controller"""
    
    def __init__(self, config_file: str = "config.json"):
        """Initialize IMU560 controller"""
        self.serial_conn: Optional[serial.Serial] = None
        self.is_connected = False
        self.continuous_mode = False
        self.continuous_thread: Optional[threading.Thread] = None
        self.continuous_callback = None
        self.stop_continuous = False
        
        # Load configuration
        self.load_config(config_file)
        self.setup_logging()
        
        logger.info("IMU560 Controller initialized")
    
    def load_config(self, config_file: str):
        """Load configuration from JSON file"""
        try:
            with open(config_file, 'r') as f:
                self.config = json.load(f)
            logger.info(f"Configuration loaded from {config_file}")
        except FileNotFoundError:
            logger.warning(f"Config file {config_file} not found, using defaults")
            self.config = {
                "serial": {"port": "/dev/ttyUSB0", "baudrate": 115200, "timeout": 1.0},
                "logging": {"level": "INFO", "format": "%(asctime)s - %(levelname)s - %(message)s"},
                "default_settings": {"continuous_mode": True, "output_frequency": 100}
            }
        except json.JSONDecodeError as e:
            logger.error(f"Invalid JSON in config file: {e}")
            raise IMUError(f"Invalid configuration file: {e}")
    
    def setup_logging(self):
        """Setup logging configuration"""
        log_config = self.config.get("logging", {})
        level = getattr(logging, log_config.get("level", "INFO"))
        format_str = log_config.get("format", "%(asctime)s - %(levelname)s - %(message)s")
        
        # Update logger configuration
        logger.setLevel(level)
        
        # Add file handler if specified
        if "file" in log_config:
            file_handler = logging.FileHandler(log_config["file"])
            file_handler.setFormatter(logging.Formatter(format_str))
            logger.addHandler(file_handler)
    
    def calculate_crc(self, data: bytes) -> int:
        """Calculate CRC16 for the given data"""
        crc = 0
        poly = 0x8408
        
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 1:
                    crc = (crc >> 1) ^ poly
                else:
                    crc >>= 1
        
        return crc & 0xFFFF
    
    def build_frame(self, cmd: int, data: bytes = b'') -> bytes:
        """Build a complete frame with CRC"""
        # Frame structure: 0xFF 0x02 CMD LEN_MSB LEN_LSB DATA CRC_MSB CRC_LSB 0x03
        data_len = len(data)
        len_msb = (data_len >> 8) & 0xFF
        len_lsb = data_len & 0xFF
        
        # Build frame without CRC
        frame_data = bytes([cmd, len_msb, len_lsb]) + data
        
        # Calculate CRC
        crc = self.calculate_crc(frame_data)
        crc_msb = (crc >> 8) & 0xFF
        crc_lsb = crc & 0xFF
        
        # Complete frame
        frame = bytes([0xFF, 0x02]) + frame_data + bytes([crc_msb, crc_lsb, 0x03])
        
        logger.debug(f"Built frame: {frame.hex().upper()}")
        return frame
    
    def parse_frame(self, data: bytes) -> Tuple[int, bytes]:
        """Parse received frame and return command and data"""
        if len(data) < 8:  # Minimum frame size
            raise IMUError("Frame too short")
        
        if data[0] != 0xFF or data[1] != 0x02:
            raise IMUError("Invalid frame sync bytes")
        
        if data[-1] != 0x03:
            raise IMUError("Invalid frame end byte")
        
        cmd = data[2]
        data_len = (data[3] << 8) | data[4]
        
        if len(data) != 8 + data_len:
            raise IMUError(f"Frame length mismatch: expected {8 + data_len}, got {len(data)}")
        
        frame_data = data[5:5+data_len]
        received_crc = (data[5+data_len] << 8) | data[6+data_len]
        
        # Verify CRC
        calculated_crc = self.calculate_crc(data[2:5+data_len])
        if received_crc != calculated_crc:
            raise IMUError(f"CRC mismatch: expected {calculated_crc:04X}, got {received_crc:04X}")
        
        logger.debug(f"Parsed frame - CMD: 0x{cmd:02X}, Data length: {data_len}")
        return cmd, frame_data
    
    def connect(self, port: Optional[str] = None, baudrate: Optional[int] = None) -> bool:
        """Connect to IMU560"""
        if port is None:
            port = self.config["serial"]["port"]
        if baudrate is None:
            baudrate = self.config["serial"]["baudrate"]
        
        try:
            self.serial_conn = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=self.config["serial"].get("timeout", 1.0),
                write_timeout=self.config["serial"].get("write_timeout", 1.0)
            )
            self.is_connected = True
            logger.info(f"Connected to IMU560 on {port} at {baudrate} baud")
            return True
        except serial.SerialException as e:
            logger.error(f"Failed to connect to IMU560: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from IMU560"""
        self.stop_continuous_mode()
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            self.is_connected = False
            logger.info("Disconnected from IMU560")
    
    def send_command(self, cmd: int, data: bytes = b'') -> Tuple[int, bytes]:
        """Send command and receive response"""
        if not self.is_connected:
            raise IMUError("Not connected to IMU")
        
        frame = self.build_frame(cmd, data)
        
        try:
            self.serial_conn.write(frame)
            logger.debug(f"Sent command 0x{cmd:02X}")
            
            # Read response
            response = self.read_frame()
            return self.parse_frame(response)
            
        except serial.SerialException as e:
            logger.error(f"Serial communication error: {e}")
            raise IMUError(f"Communication error: {e}")
    
    def read_frame(self) -> bytes:
        """Read a complete frame from serial"""
        # Look for frame start
        while True:
            byte = self.serial_conn.read(1)
            if not byte:
                raise IMUError("Timeout waiting for frame start")
            if byte[0] == 0xFF:
                break
        
        # Read second sync byte
        byte = self.serial_conn.read(1)
        if not byte or byte[0] != 0x02:
            raise IMUError("Invalid frame start sequence")
        
        # Read command and length
        cmd_len_data = self.serial_conn.read(3)
        if len(cmd_len_data) != 3:
            raise IMUError("Timeout reading command and length")
        
        data_len = (cmd_len_data[1] << 8) | cmd_len_data[2]
        
        # Read data, CRC, and end byte
        remaining = self.serial_conn.read(data_len + 3)
        if len(remaining) != data_len + 3:
            raise IMUError("Timeout reading frame data")
        
        return bytes([0xFF, 0x02]) + cmd_len_data + remaining
    
    def save_settings(self) -> bool:
        """Save current settings to EEPROM"""
        try:
            cmd, data = self.send_command(Commands.IMU_SAVE_SETTINGS)
            if cmd == Commands.IMU_ACK and len(data) == 1:
                error_code = data[0]
                if error_code == ErrorCode.NO_ERROR:
                    logger.info("Settings saved successfully")
                    return True
                else:
                    logger.error(f"Save settings failed with error code: 0x{error_code:02X}")
                    return False
            else:
                logger.error("Invalid response to save settings command")
                return False
        except Exception as e:
            logger.error(f"Error saving settings: {e}")
            return False
    
    def set_output_mask(self, mask: int) -> bool:
        """Set the default output mask"""
        try:
            # Pack mask as little-endian uint32
            data = struct.pack('<BI', 0, mask)  # Reserved byte + mask
            cmd, response = self.send_command(Commands.IMU_SET_DEFAULT_OUTPUT_MASK, data)
            
            if cmd == Commands.IMU_ACK and len(response) == 1:
                error_code = response[0]
                if error_code == ErrorCode.NO_ERROR:
                    logger.info(f"Output mask set to 0x{mask:08X}")
                    return True
                else:
                    logger.error(f"Set output mask failed with error code: 0x{error_code:02X}")
                    return False
            else:
                logger.error("Invalid response to set output mask command")
                return False
        except Exception as e:
            logger.error(f"Error setting output mask: {e}")
            return False
    
    def get_output_mask(self) -> Optional[int]:
        """Get the current output mask"""
        try:
            cmd, data = self.send_command(Commands.IMU_GET_DEFAULT_OUTPUT_MASK)
            
            if cmd == Commands.IMU_RET_DEFAULT_OUTPUT_MASK and len(data) == 4:
                mask = struct.unpack('<I', data)[0]  # Little-endian uint32
                logger.info(f"Current output mask: 0x{mask:08X}")
                return mask
            else:
                logger.error("Invalid response to get output mask command")
                return None
        except Exception as e:
            logger.error(f"Error getting output mask: {e}")
            return None
    
    def set_continuous_mode(self, enabled: bool, frequency_divider: int = 1) -> bool:
        """Set continuous mode and frequency"""
        mode = 0x01 if enabled else 0x00
        data = struct.pack('<BBB', 0, mode, frequency_divider)  # Reserved + mode + divider
        
        try:
            cmd, response = self.send_command(Commands.IMU_SET_CONTINUOUS_MODE, data)
            
            if cmd == Commands.IMU_ACK and len(response) == 1:
                error_code = response[0]
                if error_code == ErrorCode.NO_ERROR:
                    self.continuous_mode = enabled
                    freq_map = {1: 100, 2: 50, 3: 35, 4: 25, 5: 20, 6: 15, 10: 10, 20: 5, 100: 1}
                    freq = freq_map.get(frequency_divider, f"1/{frequency_divider}")
                    logger.info(f"Continuous mode {'enabled' if enabled else 'disabled'}, frequency: {freq}Hz")
                    return True
                else:
                    logger.error(f"Set continuous mode failed with error code: 0x{error_code:02X}")
                    return False
            else:
                logger.error("Invalid response to set continuous mode command")
                return False
        except Exception as e:
            logger.error(f"Error setting continuous mode: {e}")
            return False
    
    def get_continuous_mode(self) -> Optional[Tuple[bool, int]]:
        """Get current continuous mode settings"""
        try:
            cmd, data = self.send_command(Commands.IMU_GET_CONTINUOUS_MODE)
            
            if cmd == Commands.IMU_RET_CONTINUOUS_MODE and len(data) == 2:
                mode, divider = struct.unpack('<BB', data)
                enabled = mode == 0x01
                logger.info(f"Continuous mode: {'enabled' if enabled else 'disabled'}, divider: {divider}")
                return enabled, divider
            else:
                logger.error("Invalid response to get continuous mode command")
                return None
        except Exception as e:
            logger.error(f"Error getting continuous mode: {e}")
            return None
    
    def set_baudrate(self, baudrate: int) -> bool:
        """Set communication baudrate"""
        valid_rates = [9600, 19200, 38400, 57600, 115200, 230400]
        if baudrate not in valid_rates:
            logger.error(f"Invalid baudrate {baudrate}. Valid rates: {valid_rates}")
            return False
        
        try:
            data = struct.pack('<BI', 0, baudrate)  # Reserved byte + baudrate
            cmd, response = self.send_command(Commands.IMU_SET_PROTOCOL_MODE, data)
            
            if cmd == Commands.IMU_ACK and len(response) == 1:
                error_code = response[0]
                if error_code == ErrorCode.NO_ERROR:
                    logger.info(f"Baudrate set to {baudrate}")
                    return True
                else:
                    logger.error(f"Set baudrate failed with error code: 0x{error_code:02X}")
                    return False
            else:
                logger.error("Invalid response to set baudrate command")
                return False
        except Exception as e:
            logger.error(f"Error setting baudrate: {e}")
            return False
    
    def get_baudrate(self) -> Optional[int]:
        """Get current baudrate"""
        try:
            cmd, data = self.send_command(Commands.IMU_GET_PROTOCOL_MODE)
            
            if cmd == Commands.IMU_RET_PROTOCOL_MODE and len(data) == 4:
                baudrate = struct.unpack('<I', data)[0]
                logger.info(f"Current baudrate: {baudrate}")
                return baudrate
            else:
                logger.error("Invalid response to get baudrate command")
                return None
        except Exception as e:
            logger.error(f"Error getting baudrate: {e}")
            return None
    
    def set_gravity_magnitude(self, gravity: float = 9.8) -> bool:
        """Set gravity magnitude in m/s²"""
        try:
            data = struct.pack('<Bf', 0, gravity)  # Reserved byte + float32
            cmd, response = self.send_command(Commands.IMU_SET_GRAVITY_MAGNITUDE, data)
            
            if cmd == Commands.IMU_ACK and len(response) == 1:
                error_code = response[0]
                if error_code == ErrorCode.NO_ERROR:
                    logger.info(f"Gravity magnitude set to {gravity} m/s²")
                    return True
                else:
                    logger.error(f"Set gravity magnitude failed with error code: 0x{error_code:02X}")
                    return False
            else:
                logger.error("Invalid response to set gravity magnitude command")
                return False
        except Exception as e:
            logger.error(f"Error setting gravity magnitude: {e}")
            return False
    
    def get_gravity_magnitude(self) -> Optional[float]:
        """Get current gravity magnitude"""
        try:
            cmd, data = self.send_command(Commands.IMU_GET_GRAVITY_MAGNITUDE)
            
            if cmd == Commands.IMU_RET_GRAVITY_MAGNITUDE and len(data) == 4:
                gravity = struct.unpack('<f', data)[0]
                logger.info(f"Current gravity magnitude: {gravity} m/s²")
                return gravity
            else:
                logger.error("Invalid response to get gravity magnitude command")
                return None
        except Exception as e:
            logger.error(f"Error getting gravity magnitude: {e}")
            return None
    
    def set_magnetic_declination(self, declination: float) -> bool:
        """Set magnetic declination in degrees"""
        try:
            data = struct.pack('<Bf', 0, declination)  # Reserved byte + float32
            cmd, response = self.send_command(Commands.IMU_SET_MAGNETIC_DECLINATION, data)
            
            if cmd == Commands.IMU_ACK and len(response) == 1:
                error_code = response[0]
                if error_code == ErrorCode.NO_ERROR:
                    logger.info(f"Magnetic declination set to {declination}°")
                    return True
                else:
                    logger.error(f"Set magnetic declination failed with error code: 0x{error_code:02X}")
                    return False
            else:
                logger.error("Invalid response to set magnetic declination command")
                return False
        except Exception as e:
            logger.error(f"Error setting magnetic declination: {e}")
            return False
    
    def get_magnetic_declination(self) -> Optional[float]:
        """Get current magnetic declination"""
        try:
            cmd, data = self.send_command(Commands.IMU_GET_MAGNETIC_DECLINATION)
            
            if cmd == Commands.IMU_RET_MAGNETIC_DECLINATION and len(data) == 4:
                declination = struct.unpack('<f', data)[0]
                logger.info(f"Current magnetic declination: {declination}°")
                return declination
            else:
                logger.error("Invalid response to get magnetic declination command")
                return None
        except Exception as e:
            logger.error(f"Error getting magnetic declination: {e}")
            return None
    
    def start_magnetometer_calibration(self) -> bool:
        """Start magnetometer calibration"""
        try:
            data = bytes([Commands.IMU_CALIB_MAG_START])
            cmd, response = self.send_command(Commands.IMU_CALIB_MAG, data)
            
            if cmd == Commands.IMU_ACK and len(response) == 1:
                error_code = response[0]
                if error_code == ErrorCode.NO_ERROR:
                    logger.info("Magnetometer calibration started - rotate device in all directions")
                    return True
                else:
                    logger.error(f"Start calibration failed with error code: 0x{error_code:02X}")
                    return False
            else:
                logger.error("Invalid response to start calibration command")
                return False
        except Exception as e:
            logger.error(f"Error starting magnetometer calibration: {e}")
            return False
    
    def stop_magnetometer_calibration(self) -> bool:
        """Stop and save magnetometer calibration"""
        try:
            data = bytes([Commands.IMU_CALIB_MAG_STOP])
            cmd, response = self.send_command(Commands.IMU_CALIB_MAG, data)
            
            if cmd == Commands.IMU_ACK and len(response) == 1:
                error_code = response[0]
                if error_code == ErrorCode.NO_ERROR:
                    logger.info("Magnetometer calibration completed and saved")
                    return True
                else:
                    logger.error(f"Stop calibration failed with error code: 0x{error_code:02X}")
                    return False
            else:
                logger.error("Invalid response to stop calibration command")
                return False
        except Exception as e:
            logger.error(f"Error stopping magnetometer calibration: {e}")
            return False
    
    def abandon_magnetometer_calibration(self) -> bool:
        """Abandon magnetometer calibration without saving"""
        try:
            data = bytes([Commands.IMU_CALIB_MAG_ABANDON])
            cmd, response = self.send_command(Commands.IMU_CALIB_MAG, data)
            
            if cmd == Commands.IMU_ACK and len(response) == 1:
                error_code = response[0]
                if error_code == ErrorCode.NO_ERROR:
                    logger.info("Magnetometer calibration abandoned")
                    return True
                else:
                    logger.error(f"Abandon calibration failed with error code: 0x{error_code:02X}")
                    return False
            else:
                logger.error("Invalid response to abandon calibration command")
                return False
        except Exception as e:
            logger.error(f"Error abandoning magnetometer calibration: {e}")
            return False
    
    def soft_reset(self) -> bool:
        """Perform software reset"""
        try:
            cmd, response = self.send_command(Commands.IMU_SOFT_RESET)
            
            if cmd == Commands.IMU_ACK and len(response) == 1:
                error_code = response[0]
                if error_code == ErrorCode.NO_ERROR:
                    logger.info("IMU soft reset completed")
                    time.sleep(2)  # Wait for reset to complete
                    return True
                else:
                    logger.error(f"Soft reset failed with error code: 0x{error_code:02X}")
                    return False
            else:
                logger.error("Invalid response to soft reset command")
                return False
        except Exception as e:
            logger.error(f"Error performing soft reset: {e}")
            return False
    
    def parse_sensor_data(self, data: bytes, mask: int) -> SensorData:
        """Parse sensor data based on output mask"""
        sensor_data = SensorData()
        offset = 0
        
        try:
            # Parse data according to mask bits
            if mask & OutputMask.EULER:
                if len(data) >= offset + 12:
                    sensor_data.roll, sensor_data.pitch, sensor_data.yaw = struct.unpack('<fff', data[offset:offset+12])
                    offset += 12
            
            if mask & OutputMask.GYROSCOPES:
                if len(data) >= offset + 12:
                    sensor_data.gyro_x, sensor_data.gyro_y, sensor_data.gyro_z = struct.unpack('<fff', data[offset:offset+12])
                    offset += 12
            
            if mask & OutputMask.ACCELEROMETERS:
                if len(data) >= offset + 12:
                    sensor_data.accel_x, sensor_data.accel_y, sensor_data.accel_z = struct.unpack('<fff', data[offset:offset+12])
                    offset += 12
            
            if mask & OutputMask.MAGNETOMETERS:
                if len(data) >= offset + 12:
                    sensor_data.mag_x, sensor_data.mag_y, sensor_data.mag_z = struct.unpack('<fff', data[offset:offset+12])
                    offset += 12
            
            if mask & OutputMask.TEMPERATURES:
                if len(data) >= offset + 8:
                    sensor_data.temp_0, sensor_data.temp_1 = struct.unpack('<ff', data[offset:offset+8])
                    offset += 8
            
            if mask & OutputMask.TIME_SINCE_RESET:
                if len(data) >= offset + 4:
                    sensor_data.time_since_reset = struct.unpack('<I', data[offset:offset+4])[0]
                    offset += 4
            
            if mask & OutputMask.GPS_INFO:
                if len(data) >= offset + 6:
                    sensor_data.gps_itow = struct.unpack('<I', data[offset:offset+4])[0]
                    sensor_data.gps_flags = data[offset+4]
                    sensor_data.gps_num_sv = data[offset+5]
                    offset += 6
            
            if mask & OutputMask.BARO_ALTITUDE:
                if len(data) >= offset + 4:
                    sensor_data.baro_altitude = struct.unpack('<i', data[offset:offset+4])[0]
                    offset += 4
            
            if mask & OutputMask.BARO_PRESSURE:
                if len(data) >= offset + 4:
                    sensor_data.baro_pressure = struct.unpack('<I', data[offset:offset+4])[0]
                    offset += 4
            
            if mask & OutputMask.POSITION:
                if len(data) >= offset + 24:
                    sensor_data.latitude, sensor_data.longitude, sensor_data.altitude = struct.unpack('<ddd', data[offset:offset+24])
                    offset += 24
            
            if mask & OutputMask.VELOCITY:
                if len(data) >= offset + 12:
                    sensor_data.vel_north, sensor_data.vel_east, sensor_data.vel_down = struct.unpack('<fff', data[offset:offset+12])
                    offset += 12
            
            if mask & OutputMask.UTC_TIME_REFERENCE:
                if len(data) >= offset + 7:
                    sensor_data.utc_year = struct.unpack('<H', data[offset:offset+2])[0]
                    sensor_data.utc_month = data[offset+2]
                    sensor_data.utc_day = data[offset+3]
                    sensor_data.utc_hour = data[offset+4]
                    sensor_data.utc_minute = data[offset+5]
                    sensor_data.utc_second = data[offset+6]
                    offset += 7
            
            if mask & OutputMask.MAG_HEADING:
                if len(data) >= offset + 4:
                    sensor_data.mag_heading = struct.unpack('<f', data[offset:offset+4])[0]
                    offset += 4
            
        except struct.error as e:
            logger.error(f"Error parsing sensor data: {e}")
        
        return sensor_data
    
    def get_sensor_data(self, mask: Optional[int] = None) -> Optional[SensorData]:
        """Get sensor data with specified mask or default"""
        try:
            if mask is None:
                cmd, data = self.send_command(Commands.IMU_GET_DEFAULT_OUTPUT)
                expected_cmd = Commands.IMU_RET_DEFAULT_OUTPUT
                current_mask = self.get_output_mask()
                if current_mask is None:
                    logger.error("Failed to get current output mask")
                    return None
            else:
                mask_data = struct.pack('<I', mask)
                cmd, data = self.send_command(Commands.IMU_GET_SPECIFIC_OUTPUT, mask_data)
                expected_cmd = Commands.IMU_RET_SPECIFIC_OUTPUT
                current_mask = mask
            
            if cmd == expected_cmd:
                return self.parse_sensor_data(data, current_mask)
            else:
                logger.error(f"Unexpected response command: 0x{cmd:02X}")
                return None
                
        except Exception as e:
            logger.error(f"Error getting sensor data: {e}")
            return None
    
    def start_continuous_mode(self, callback=None, frequency_divider: int = 1):
        """Start continuous data output mode"""
        if not self.set_continuous_mode(True, frequency_divider):
            logger.error("Failed to enable continuous mode")
            return False
        
        self.continuous_callback = callback
        self.stop_continuous = False
        self.continuous_thread = threading.Thread(target=self._continuous_reader)
        self.continuous_thread.daemon = True
        self.continuous_thread.start()
        logger.info("Continuous mode started")
        return True
    
    def stop_continuous_mode(self):
        """Stop continuous data output mode"""
        self.stop_continuous = True
        if self.continuous_thread and self.continuous_thread.is_alive():
            self.continuous_thread.join(timeout=2)
        
        if self.is_connected:
            self.set_continuous_mode(False)
        
        logger.info("Continuous mode stopped")
    
    def _continuous_reader(self):
        """Internal thread function for continuous data reading"""
        current_mask = self.get_output_mask()
        if current_mask is None:
            logger.error("Cannot get output mask for continuous mode")
            return
        
        while not self.stop_continuous and self.is_connected:
            try:
                frame = self.read_frame()
                cmd, data = self.parse_frame(frame)
                
                if cmd == Commands.IMU_CONTINUOUS_DEFAULT_OUTPUT:
                    sensor_data = self.parse_sensor_data(data, current_mask)
                    if self.continuous_callback:
                        self.continuous_callback(sensor_data)
                
            except Exception as e:
                if not self.stop_continuous:
                    logger.error(f"Error in continuous reader: {e}")
                    time.sleep(0.1)  # Brief pause before retry
    
    def __enter__(self):
        """Context manager entry"""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.disconnect()

# Default output mask with commonly used sensors
DEFAULT_OUTPUT_MASK = (
    OutputMask.EULER |
    OutputMask.GYROSCOPES |
    OutputMask.ACCELEROMETERS |
    OutputMask.MAGNETOMETERS |
    OutputMask.TEMPERATURES |
    OutputMask.TIME_SINCE_RESET |
    OutputMask.GPS_INFO |
    OutputMask.BARO_ALTITUDE |
    OutputMask.BARO_PRESSURE |
    OutputMask.POSITION |
    OutputMask.VELOCITY |
    OutputMask.UTC_TIME_REFERENCE |
    OutputMask.MAG_HEADING
)

if __name__ == "__main__":
    # Simple test
    with IMU560() as imu:
        if imu.connect():
            print("Connected to IMU560")
            
            # Get current settings
            mask = imu.get_output_mask()
            print(f"Current output mask: 0x{mask:08X}")
            
            # Get sensor data
            data = imu.get_sensor_data()
            if data:
                print(f"Roll: {data.roll:.3f}, Pitch: {data.pitch:.3f}, Yaw: {data.yaw:.3f}")
                print(f"Accel: ({data.accel_x:.3f}, {data.accel_y:.3f}, {data.accel_z:.3f}) m/s²")
                print(f"Gyro: ({data.gyro_x:.3f}, {data.gyro_y:.3f}, {data.gyro_z:.3f}) rad/s")
        else:
            print("Failed to connect to IMU560")
        
                        