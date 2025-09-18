import serial
import json
import time
import logging
import struct
from typing import Optional, Dict, Any, Tuple, List
from threading import Thread, Event
import signal
import sys

class IMUController:
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 2.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None
        self.running = False
        self.data_thread = None
        self.stop_event = Event()
        
    def connect(self) -> bool:
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=8,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout
            )
            logging.info(f"IMU connected on {self.port} at {self.baudrate} baud")
            return True
        except Exception as e:
            logging.error(f"Failed to connect to IMU: {e}")
            return False
    
    def calc_crc(self, data: bytes) -> int:
        """Calculate CRC using the polynomial 0x8408"""
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
    
    def send_command(self, cmd: int, data: bytes = b'') -> bool:
        if not self.serial or not self.serial.is_open:
            return False
        
        try:
            # Frame format: 0xFF, 0x02, CMD, LEN_MSB, LEN_LSB, DATA, CRC_MSB, CRC_LSB, 0x03
            data_len = len(data)
            len_msb = (data_len >> 8) & 0xFF
            len_lsb = data_len & 0xFF
            
            # Build frame without CRC
            frame_data = bytes([cmd, len_msb, len_lsb]) + data
            crc = self.calc_crc(frame_data)
            
            frame = bytes([0xFF, 0x02]) + frame_data + struct.pack('<H', crc) + bytes([0x03])
            
            self.serial.write(frame)
            self.serial.flush()
            return True
        except Exception as e:
            logging.error(f"Failed to send command 0x{cmd:02X}: {e}")
            return False
    
    def read_response(self) -> Optional[Dict]:
        if not self.serial or not self.serial.is_open:
            return None
        
        try:
            # Look for frame start
            while True:
                byte = self.serial.read(1)
                if not byte:
                    return None
                if byte == b'\xFF':
                    break
            
            # Read frame start byte
            start_byte = self.serial.read(1)
            if start_byte != b'\x02':
                return None
            
            # Read CMD, LEN, DATA, CRC, END
            header = self.serial.read(3)  # CMD + LEN_MSB + LEN_LSB
            if len(header) != 3:
                return None
            
            cmd, len_msb, len_lsb = header
            data_len = (len_msb << 8) | len_lsb
            
            data = self.serial.read(data_len) if data_len > 0 else b''
            crc_bytes = self.serial.read(2)
            end_byte = self.serial.read(1)
            
            if end_byte != b'\x03':
                return None
            
            # Verify CRC
            frame_data = bytes([cmd, len_msb, len_lsb]) + data
            calculated_crc = self.calc_crc(frame_data)
            received_crc = struct.unpack('<H', crc_bytes)[0]
            
            if calculated_crc != received_crc:
                logging.warning("CRC mismatch in response")
            
            return {
                'cmd': cmd,
                'data': data,
                'crc_valid': calculated_crc == received_crc
            }
        except Exception as e:
            logging.error(f"Failed to read response: {e}")
            return None
    
    def start_magnetic_calibration(self) -> bool:
        """Start magnetic field calibration (0x70)"""
        success = self.send_command(0x70, bytes([0x08]))  # DATA = 0x08
        if success:
            response = self.read_response()
            if response and response['cmd'] == 0x01:  # ACK
                logging.info("Magnetic calibration started")
                return True
        logging.error("Failed to start magnetic calibration")
        return False
    
    def stop_magnetic_calibration(self) -> bool:
        """Stop magnetic field calibration (0x70)"""
        success = self.send_command(0x70, bytes([0x0A]))  # DATA = 0x0A
        if success:
            response = self.read_response()
            if response and response['cmd'] == 0x01:  # ACK
                logging.info("Magnetic calibration stopped")
                return True
        logging.error("Failed to stop magnetic calibration")
        return False
    
    def abandon_magnetic_calibration(self) -> bool:
        """Abandon magnetic field calibration (0x70)"""
        success = self.send_command(0x70, bytes([0x09]))  # DATA = 0x09
        if success:
            response = self.read_response()
            if response and response['cmd'] == 0x01:  # ACK
                logging.info("Magnetic calibration abandoned")
                return True
        logging.error("Failed to abandon magnetic calibration")
        return False
    
    def set_continuous_mode(self, frequency: int = 10) -> bool:
        """Set continuous output mode"""
        # Map frequency to divider
        freq_map = {100: 1, 50: 2, 35: 3, 25: 4, 20: 5, 15: 6, 10: 10, 5: 20, 1: 100}
        divider = freq_map.get(frequency, 10)
        
        data = bytes([0x00, 0x01, divider])  # Mode=1 (continuous), Divider
        success = self.send_command(0x53, data)
        if success:
            response = self.read_response()
            if response and response['cmd'] == 0x01:  # ACK
                logging.info(f"Continuous mode set to {frequency}Hz")
                return True
        logging.error("Failed to set continuous mode")
        return False
    
    def get_sensor_data(self) -> Optional[Dict]:
        """Get sensor data"""
        success = self.send_command(0x56)  # Get default output
        if success:
            response = self.read_response()
            if response and response['cmd'] == 0x57:  # RET_DEFAULT_OUTPUT
                return self.parse_sensor_data(response['data'])
        return None
    
    def parse_sensor_data(self, data: bytes) -> Dict:
        """Parse sensor data from response"""
        if len(data) < 36:  # Minimum expected data size
            return {}
        
        # Parse according to the data format (little endian)
        parsed = {}
        offset = 0
        
        # Roll, Pitch, Yaw (4 bytes each, float)
        parsed['roll'] = struct.unpack('<f', data[offset:offset+4])[0]
        offset += 4
        parsed['pitch'] = struct.unpack('<f', data[offset:offset+4])[0]
        offset += 4
        parsed['yaw'] = struct.unpack('<f', data[offset:offset+4])[0]
        offset += 4
        
        # Gyroscope (Gx, Gy, Gz)
        parsed['gx'] = struct.unpack('<f', data[offset:offset+4])[0]
        offset += 4
        parsed['gy'] = struct.unpack('<f', data[offset:offset+4])[0]
        offset += 4
        parsed['gz'] = struct.unpack('<f', data[offset:offset+4])[0]
        offset += 4
        
        # Accelerometer (Ax, Ay, Az)
        parsed['ax'] = struct.unpack('<f', data[offset:offset+4])[0]
        offset += 4
        parsed['ay'] = struct.unpack('<f', data[offset:offset+4])[0]
        offset += 4
        parsed['az'] = struct.unpack('<f', data[offset:offset+4])[0]
        offset += 4
        
        return parsed
    
    def close(self):
        self.stop_event.set()
        if self.data_thread:
            self.data_thread.join()
        if self.serial and self.serial.is_open:
            self.serial.close()

class SP2520Controller:
    def __init__(self, port='/dev/ttyUSB1', baudrate=2400, address=1):
        self.address = address
        self.pan_speed = 0x20
        self.tilt_speed = 0x20
        
        self.serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=8,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )
    
    def send_command(self, cmd1, cmd2, data1, data2):
        checksum = (self.address + cmd1 + cmd2 + data1 + data2) % 256
        command = bytes([0xFF, self.address, cmd1, cmd2, data1, data2, checksum])
        self.serial.write(command)
    
    def stop(self):
        self.send_command(0x00, 0x00, 0x00, 0x00)
    
    def move_left(self):
        self.send_command(0x00, 0x04, self.pan_speed, 0x00)
    
    def move_right(self):
        self.send_command(0x00, 0x02, self.pan_speed, 0x00)
    
    def move_up(self):
        self.send_command(0x00, 0x08, 0x00, self.tilt_speed)
    
    def move_down(self):
        self.send_command(0x00, 0x10, 0x00, self.tilt_speed)
    
    def set_speed(self, pan_speed=None, tilt_speed=None):
        if pan_speed is not None:
            self.pan_speed = max(0, min(0x40, pan_speed))
        if tilt_speed is not None:
            self.tilt_speed = max(0, min(0x40, tilt_speed))
    
    def close(self):
        if self.serial and self.serial.is_open:
            self.stop()
            self.serial.close()

class IMUCalibrationSystem:
    def __init__(self, config_file: str = 'config.json'):
        self.config = self.load_config(config_file)
        self.setup_logging()
        
        self.imu = None
        self.pantilt = None
        self.calibration_running = False
        
    def load_config(self, config_file: str) -> Dict[str, Any]:
        try:
            with open(config_file, 'r') as f:
                return json.load(f)
        except Exception as e:
            print(f"Failed to load config: {e}")
            sys.exit(1)
    
    def setup_logging(self):
        log_config = self.config.get('logging', {})
        level = getattr(logging, log_config.get('level', 'INFO'))
        log_file = log_config.get('file', 'imu_calibration.log')
        
        logging.basicConfig(
            level=level,
            format='%(asctime)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler(log_file),
                logging.StreamHandler(sys.stdout)
            ]
        )
    
    def initialize_devices(self) -> bool:
        """Initialize IMU and Pan-Tilt controllers"""
        try:
            # Initialize IMU
            imu_config = self.config['imu']
            self.imu = IMUController(
                port=imu_config['port'],
                baudrate=imu_config['baudrate'],
                timeout=imu_config['timeout']
            )
            
            if not self.imu.connect():
                return False
            
            # Initialize Pan-Tilt
            pantilt_config = self.config['pantilt']
            self.pantilt = SP2520Controller(
                port=pantilt_config['port'],
                baudrate=pantilt_config['baudrate'],
                address=pantilt_config['address']
            )
            
            self.pantilt.set_speed(
                pan_speed=pantilt_config['pan_speed'],
                tilt_speed=pantilt_config['tilt_speed']
            )
            
            logging.info("All devices initialized successfully")
            return True
            
        except Exception as e:
            logging.error(f"Failed to initialize devices: {e}")
            return False
    
    def perform_calibration(self) -> bool:
        """Perform complete IMU magnetic calibration"""
        if not self.initialize_devices():
            return False
        
        try:
            self.calibration_running = True
            cal_config = self.config['calibration']
            
            # Start magnetic calibration
            if not self.imu.start_magnetic_calibration():
                return False
            
            # Set continuous mode for real-time monitoring
            self.imu.set_continuous_mode(cal_config['continuous_mode_frequency'])
            
            # Perform rotation calibration at different tilt angles
            for tilt_angle in cal_config['tilt_positions']:
                logging.info(f"Calibrating at tilt angle: {tilt_angle} degrees")
                
                # Move to tilt position
                self.move_to_tilt_position(tilt_angle)
                time.sleep(2)  # Wait for stabilization
                
                # Perform 360-degree rotation
                if not self.perform_rotation_calibration(cal_config):
                    self.imu.abandon_magnetic_calibration()
                    return False
                
                # Return to center position
                self.pantilt.stop()
                time.sleep(1)
            
            # Stop magnetic calibration (saves parameters)
            if not self.imu.stop_magnetic_calibration():
                logging.error("Failed to stop calibration properly")
                return False
            
            logging.info("IMU magnetic calibration completed successfully")
            return True
            
        except Exception as e:
            logging.error(f"Calibration failed: {e}")
            if self.imu:
                self.imu.abandon_magnetic_calibration()
            return False
        
        finally:
            self.calibration_running = False
            self.cleanup()
    
    def move_to_tilt_position(self, angle: float):
        """Move pan-tilt to specific tilt position"""
        # Simple tilt positioning (you may need to adjust based on your specific setup)
        if angle > 0:
            # Tilt up
            duration = abs(angle) / 45.0  # Rough estimation
            self.pantilt.move_up()
            time.sleep(duration)
        elif angle < 0:
            # Tilt down
            duration = abs(angle) / 45.0
            self.pantilt.move_down()
            time.sleep(duration)
        
        self.pantilt.stop()
    
    def perform_rotation_calibration(self, cal_config: Dict) -> bool:
        """Perform 360-degree rotation for magnetic calibration"""
        rotation_time = cal_config['rotation_time']
        rotation_steps = cal_config['rotation_steps']
        step_delay = cal_config['step_delay']
        
        # Calculate time per step
        step_time = rotation_time / rotation_steps
        
        logging.info(f"Starting 360-degree rotation ({rotation_steps} steps)")
        
        for step in range(rotation_steps):
            if not self.calibration_running:
                return False
            
            # Rotate right for calculated time
            self.pantilt.move_right()
            time.sleep(step_time)
            self.pantilt.stop()
            
            # Wait and collect data
            time.sleep(step_delay)
            
            # Get sensor data for monitoring
            data = self.imu.get_sensor_data()
            if data:
                logging.debug(f"Step {step+1}/{rotation_steps}: "
                            f"Yaw={data.get('yaw', 0):.1f}°, "
                            f"Mag=({data.get('mx', 0):.2f}, {data.get('my', 0):.2f}, {data.get('mz', 0):.2f})")
            
            # Check for timeout
            if step * (step_time + step_delay) > cal_config['calibration_timeout']:
                logging.error("Calibration timeout reached")
                return False
        
        logging.info("360-degree rotation completed")
        return True
    
    def monitor_calibration_progress(self):
        """Monitor calibration progress in real-time"""
        while self.calibration_running:
            data = self.imu.get_sensor_data()
            if data:
                print(f"\rYaw: {data.get('yaw', 0):6.1f}° | "
                      f"Roll: {data.get('roll', 0):6.1f}° | "
                      f"Pitch: {data.get('pitch', 0):6.1f}°", end='')
            time.sleep(0.1)
        print()  # New line after monitoring
    
    def test_imu_connection(self) -> bool:
        """Test IMU connection and basic functionality"""
        if not self.initialize_devices():
            return False
        
        try:
            # Get sensor data
            data = self.imu.get_sensor_data()
            if data:
                logging.info("IMU Test Results:")
                logging.info(f"  Roll: {data.get('roll', 0):.2f}°")
                logging.info(f"  Pitch: {data.get('pitch', 0):.2f}°")
                logging.info(f"  Yaw: {data.get('yaw', 0):.2f}°")
                logging.info(f"  Gyro: ({data.get('gx', 0):.2f}, {data.get('gy', 0):.2f}, {data.get('gz', 0):.2f})")
                logging.info(f"  Accel: ({data.get('ax', 0):.2f}, {data.get('ay', 0):.2f}, {data.get('az', 0):.2f})")
                return True
            else:
                logging.error("Failed to get sensor data")
                return False
                
        except Exception as e:
            logging.error(f"IMU test failed: {e}")
            return False
        finally:
            self.cleanup()
    
    def test_pantilt_connection(self) -> bool:
        """Test pan-tilt connection and basic movements"""
        if not self.initialize_devices():
            return False
        
        try:
            logging.info("Testing pan-tilt movements...")
            
            # Test each direction
            movements = [
                ("left", self.pantilt.move_left),
                ("right", self.pantilt.move_right),
                ("up", self.pantilt.move_up),
                ("down", self.pantilt.move_down)
            ]
            
            for direction, move_func in movements:
                logging.info(f"Testing {direction} movement")
                move_func()
                time.sleep(1)
                self.pantilt.stop()
                time.sleep(1)
            
            logging.info("Pan-tilt test completed successfully")
            return True
            
        except Exception as e:
            logging.error(f"Pan-tilt test failed: {e}")
            return False
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        try:
            if self.pantilt:
                self.pantilt.close()
            if self.imu:
                self.imu.close()
            logging.info("Cleanup completed")
        except Exception as e:
            logging.error(f"Cleanup error: {e}")
    
    def signal_handler(self, signum, frame):
        """Handle interrupt signals"""
        logging.info("Received interrupt signal, stopping calibration...")
        self.calibration_running = False
        if self.imu:
            self.imu.abandon_magnetic_calibration()
        self.cleanup()
        sys.exit(0)

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='IMU Calibration System')
    parser.add_argument('--config', default='config.json', help='Configuration file path')
    parser.add_argument('--test-imu', action='store_true', help='Test IMU connection only')
    parser.add_argument('--test-pantilt', action='store_true', help='Test pan-tilt connection only')
    parser.add_argument('--calibrate', action='store_true', help='Perform full calibration')
    
    args = parser.parse_args()
    
    # Create calibration system
    cal_system = IMUCalibrationSystem(args.config)
    
    # Set up signal handlers
    signal.signal(signal.SIGINT, cal_system.signal_handler)
    signal.signal(signal.SIGTERM, cal_system.signal_handler)
    
    try:
        if args.test_imu:
            print("Testing IMU connection...")
            success = cal_system.test_imu_connection()
            print(f"IMU test: {'PASSED' if success else 'FAILED'}")
        
        elif args.test_pantilt:
            print("Testing Pan-Tilt connection...")
            success = cal_system.test_pantilt_connection()
            print(f"Pan-Tilt test: {'PASSED' if success else 'FAILED'}")
        
        elif args.calibrate:
            print("Starting IMU magnetic calibration...")
            print("Make sure the IMU is mounted on the pan-tilt system.")
            print("Calibration will take approximately 2-3 minutes.")
            input("Press Enter to continue or Ctrl+C to cancel...")
            
            # Start monitoring in separate thread
            from threading import Thread
            monitor_thread = Thread(target=cal_system.monitor_calibration_progress)
            monitor_thread.daemon = True
            monitor_thread.start()
            
            success = cal_system.perform_calibration()
            print(f"\nCalibration: {'COMPLETED' if success else 'FAILED'}")
            
            if success:
                print("IMU magnetic calibration completed successfully!")
                print("The calibration parameters have been saved to the IMU.")
            else:
                print("Calibration failed. Check logs for details.")
        
        else:
            parser.print_help()
    
    except KeyboardInterrupt:
        print("\nOperation cancelled by user")
    except Exception as e:
        print(f"Error: {e}")
        logging.error(f"Main error: {e}")

if __name__ == "__main__":
    main()         