#!/usr/bin/env python3

import serial
import time
import struct
import sys
import threading
from math import fabs

class NorthPointingSystem:
    def __init__(self, heading_port='COM9', controller_port='/dev/ttyUSB1', 
                 heading_baudrate=115200, controller_baudrate=2400, controller_address=1):
        
        # Heading Monitor Setup
        self.heading_port = heading_port
        self.heading_baudrate = heading_baudrate
        self.heading_ser = None
        self.heading_position = 40
        
        # SP2520 Controller Setup
        self.controller_address = controller_address
        self.pan_speed = 0x15  # Slower speed for precise north pointing
        self.tilt_speed = 0x15
        
        self.controller_ser = serial.Serial(
            port=controller_port,
            baudrate=controller_baudrate,
            bytesize=8,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )
        
        # Control parameters
        self.target_heading = 0.0  # North
        self.tolerance = 2.0       # Degrees tolerance
        self.current_heading = None
        self.last_adjustment_time = 0
        self.adjustment_cooldown = 0.5  # Seconds between adjustments
        self.running = False
        
    def connect_heading_monitor(self):
        """Connect to heading monitor"""
        try:
            self.heading_ser = serial.Serial(self.heading_port, self.heading_baudrate, timeout=1)
            print(f"✓ Connected to heading monitor on {self.heading_port}")
            time.sleep(1)
            return True
        except Exception as e:
            print(f"✗ Heading monitor connection failed: {e}")
            return False
    
    def read_frame(self):
        """Read a complete data frame from heading monitor"""
        try:
            self.heading_ser.flushInput()
            
            while True:
                byte = self.heading_ser.read(1)
                if byte == b'\xFF':
                    next_byte = self.heading_ser.read(1)
                    if next_byte == b'\x02':
                        break
            
            header = self.heading_ser.read(3)
            if len(header) != 3:
                return None
            
            cmd = header[0]
            data_len = (header[1] << 8) | header[2]
            
            data = self.heading_ser.read(data_len)
            if len(data) != data_len:
                return None
            
            trailer = self.heading_ser.read(3)
            if len(trailer) != 3 or trailer[2] != 0x03:
                return None
            
            return {'cmd': cmd, 'data': data}
            
        except Exception as e:
            return None
    
    def extract_magnetic_heading(self, data):
        """Extract magnetic heading from position 40"""
        if len(data) >= self.heading_position + 4:
            try:
                heading = struct.unpack('<f', data[self.heading_position:self.heading_position+4])[0]
                # Normalize heading to 0-360 range
                heading = heading % 360.0
                return heading
            except:
                return None
        return None
    
    def send_command(self, cmd1, cmd2, data1, data2):
        """Send command to SP2520 controller"""
        checksum = (self.controller_address + cmd1 + cmd2 + data1 + data2) % 256
        command = bytes([0xFF, self.controller_address, cmd1, cmd2, data1, data2, checksum])
        self.controller_ser.write(command)
    
    def stop_movement(self):
        """Stop all movement"""
        self.send_command(0x00, 0x00, 0x00, 0x00)
    
    def move_left(self, speed=None):
        """Move left (counter-clockwise)"""
        if speed is None:
            speed = self.pan_speed
        self.send_command(0x00, 0x04, speed, 0x00)
    
    def move_right(self, speed=None):
        """Move right (clockwise)"""
        if speed is None:
            speed = self.pan_speed
        self.send_command(0x00, 0x02, speed, 0x00)
    
    def calculate_heading_error(self, current, target):
        """Calculate the shortest angular distance to target"""
        error = target - current
        
        # Normalize to -180 to +180 range
        while error > 180:
            error -= 360
        while error < -180:
            error += 360
            
        return error
    
    def adjust_to_north(self):
        """Adjust the machine to point north"""
        if self.current_heading is None:
            return
        
        current_time = time.time()
        if current_time - self.last_adjustment_time < self.adjustment_cooldown:
            return
        
        error = self.calculate_heading_error(self.current_heading, self.target_heading)
        
        if fabs(error) <= self.tolerance:
            self.stop_movement()
            return
        
        # Determine movement direction and speed
        if error > 0:
            # Need to turn right (clockwise) to reach north
            if fabs(error) > 10:
                speed = self.pan_speed
            else:
                speed = max(0x08, int(self.pan_speed * fabs(error) / 10))
            self.move_right(speed)
            print(f"→ Turning RIGHT, Error: {error:.1f}°, Speed: {speed}")
        else:
            # Need to turn left (counter-clockwise) to reach north
            if fabs(error) > 10:
                speed = self.pan_speed
            else:
                speed = max(0x08, int(self.pan_speed * fabs(error) / 10))
            self.move_left(speed)
            print(f"← Turning LEFT, Error: {error:.1f}°, Speed: {speed}")
        
        self.last_adjustment_time = current_time
    
    def heading_monitor_thread(self):
        """Thread function to continuously read heading"""
        while self.running:
            frame = self.read_frame()
            
            if frame and frame['cmd'] == 0x90:
                heading = self.extract_magnetic_heading(frame['data'])
                if heading is not None:
                    self.current_heading = heading
            
            time.sleep(0.05)  # 20Hz update rate
    
    def start_north_pointing(self):
        """Start the north pointing system"""
        print("=" * 60)
        print("         NORTH POINTING SYSTEM")
        print("=" * 60)
        print(f"Target: North (0°)")
        print(f"Tolerance: ±{self.tolerance}°")
        print(f"Pan Speed: {self.pan_speed}")
        print("Press Ctrl+C to stop\n")
        
        if not self.connect_heading_monitor():
            return
        
        self.running = True
        
        # Start heading monitoring thread
        heading_thread = threading.Thread(target=self.heading_monitor_thread, daemon=True)
        heading_thread.start()
        
        # Wait for first heading reading
        print("Waiting for heading data...")
        while self.current_heading is None and self.running:
            time.sleep(0.1)
        
        if not self.running:
            return
        
        print(f"Initial heading: {self.current_heading:.1f}°")
        
        try:
            while self.running:
                if self.current_heading is not None:
                    error = self.calculate_heading_error(self.current_heading, self.target_heading)
                    status = "ON TARGET" if fabs(error) <= self.tolerance else "ADJUSTING"
                    
                    timestamp = time.strftime("%H:%M:%S")
                    print(f"\r[{timestamp}] Heading: {self.current_heading:6.1f}° | "
                          f"Error: {error:+6.1f}° | {status}", end="", flush=True)
                    
                    self.adjust_to_north()
                
                time.sleep(0.1)
        
        except KeyboardInterrupt:
            print(f"\n\nNorth pointing system stopped by user")
            if self.current_heading is not None:
                final_error = self.calculate_heading_error(self.current_heading, self.target_heading)
                print(f"Final heading: {self.current_heading:.1f}°")
                print(f"Final error: {final_error:.1f}°")
        
        finally:
            self.cleanup()
    
    def calibrate_and_point_north(self):
        """Calibration mode - rotate 360° then point north"""
        print("=" * 60)
        print("         CALIBRATION & NORTH POINTING")
        print("=" * 60)
        
        if not self.connect_heading_monitor():
            return
        
        self.running = True
        
        # Start heading monitoring thread
        heading_thread = threading.Thread(target=self.heading_monitor_thread, daemon=True)
        heading_thread.start()
        
        # Wait for first heading reading
        while self.current_heading is None:
            time.sleep(0.1)
        
        initial_heading = self.current_heading
        print(f"Starting calibration from: {initial_heading:.1f}°")
        print("Performing 360° rotation for calibration...")
        
        # Rotate 360 degrees for calibration
        start_time = time.time()
        self.move_right(0x10)  # Slow speed for calibration
        
        try:
            while time.time() - start_time < 30:  # Max 30 seconds
                if self.current_heading is not None:
                    print(f"\rCalibrating... Heading: {self.current_heading:6.1f}°", end="", flush=True)
                    
                    # Check if we've completed roughly 360°
                    heading_change = abs(self.current_heading - initial_heading)
                    if heading_change > 350 or (heading_change < 10 and time.time() - start_time > 15):
                        break
                
                time.sleep(0.1)
            
            print(f"\nCalibration complete. Now pointing to North...")
            self.stop_movement()
            time.sleep(1)
            
            # Now point to north
            self.start_north_pointing()
            
        except KeyboardInterrupt:
            print("\nCalibration interrupted")
            self.cleanup()
    
    def cleanup(self):
        """Clean up connections"""
        self.running = False
        
        try:
            self.stop_movement()
            time.sleep(0.5)
        except:
            pass
        
        if self.heading_ser and self.heading_ser.is_open:
            self.heading_ser.close()
        
        if self.controller_ser and self.controller_ser.is_open:
            self.controller_ser.close()
        
        print("\nConnections closed")

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='North Pointing System')
    parser.add_argument('--heading-port', default='COM9', help='Heading monitor serial port')
    parser.add_argument('--controller-port', default='/dev/ttyUSB1', help='Controller serial port')
    parser.add_argument('--tolerance', type=float, default=2.0, help='Pointing tolerance in degrees')
    parser.add_argument('--speed', type=int, default=0x15, help='Movement speed (hex value)')
    parser.add_argument('--calibrate', action='store_true', help='Perform 360° calibration first')
    
    args = parser.parse_args()
    
    system = NorthPointingSystem(
        heading_port=args.heading_port,
        controller_port=args.controller_port
    )
    
    system.tolerance = args.tolerance
    system.pan_speed = args.speed
    
    if args.calibrate:
        system.calibrate_and_point_north()
    else:
        system.start_north_pointing()

if __name__ == "__main__":
    main()