#!/usr/bin/env python3

import serial
import time
import struct
import sys

class FinalHeadingMonitor:
    def __init__(self, port='COM9', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.heading_position = 40  # Confirmed position for magnetic heading
    
    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            print(f"✓ Connected to {self.port}")
            time.sleep(1)
            return True
        except Exception as e:
            print(f"✗ Connection failed: {e}")
            return False
    
    def read_frame(self):
        """Read a complete data frame"""
        try:
            # Clear buffer and find frame start
            self.ser.flushInput()
            
            while True:
                byte = self.ser.read(1)
                if byte == b'\xFF':
                    next_byte = self.ser.read(1)
                    if next_byte == b'\x02':
                        break
            
            # Read header: CMD + LEN_MSB + LEN_LSB
            header = self.ser.read(3)
            if len(header) != 3:
                return None
            
            cmd = header[0]
            data_len = (header[1] << 8) | header[2]
            
            # Read data payload
            data = self.ser.read(data_len)
            if len(data) != data_len:
                return None
            
            # Read trailer (CRC + end byte)
            trailer = self.ser.read(3)
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
                return heading
            except:
                return None
        return None
    
    def monitor_heading(self):
        """Monitor magnetic heading continuously"""
        print("=" * 60)
        print("         MAGNETIC HEADING MONITOR")
        print("=" * 60)
        print(f"Reading magnetic heading from position {self.heading_position}")
        print("Press Ctrl+C to stop\n")
        
        if not self.connect():
            return
        
        reading_count = 0
        successful_readings = 0
        last_heading = None
        
        try:
            while True:
                frame = self.read_frame()
                
                if frame and frame['cmd'] == 0x90:  # Continuous data frame
                    heading = self.extract_magnetic_heading(frame['data'])
                    
                    if heading is not None:
                        timestamp = time.strftime("%H:%M:%S")
                        
                        # Show change indicator
                        change_indicator = ""
                        if last_heading is not None:
                            diff = abs(heading - last_heading)
                            if diff > 1.0:
                                change_indicator = " ↑" if heading > last_heading else " ↓"
                            elif diff > 0.1:
                                change_indicator = " ~"
                        
                        print(f"\r[{timestamp}] Mag.Head: {heading:8.3f}°{change_indicator}", end="", flush=True)
                        
                        last_heading = heading
                        successful_readings += 1
                    else:
                        print("E", end="", flush=True)  # Error reading
                
                reading_count += 1
                time.sleep(0.05)  # 20Hz update rate
        
        except KeyboardInterrupt:
            print(f"\n\nMonitoring stopped by user")
            print(f"Total frames processed: {reading_count}")
            print(f"Successful readings: {successful_readings}")
            if reading_count > 0:
                success_rate = (successful_readings / reading_count) * 100
                print(f"Success rate: {success_rate:.1f}%")
            if last_heading is not None:
                print(f"Last magnetic heading: {last_heading:.3f}°")
        
        finally:
            self.cleanup()
    
    def test_heading(self):
        """Quick test to verify heading reading"""
        print("Testing magnetic heading reading...")
        
        if not self.connect():
            return
        
        for i in range(5):
            frame = self.read_frame()
            if frame and frame['cmd'] == 0x90:
                heading = self.extract_magnetic_heading(frame['data'])
                if heading is not None:
                    print(f"Test {i+1}: Magnetic Heading = {heading:.3f}°")
                else:
                    print(f"Test {i+1}: Failed to read heading")
            time.sleep(0.2)
        
        self.cleanup()
    
    def cleanup(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        print("\nConnection closed")

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Final Magnetic Heading Monitor')
    parser.add_argument('--port', default='COM9', help='Serial port')
    parser.add_argument('--test', action='store_true', help='Quick test mode')
    
    args = parser.parse_args()
    
    monitor = FinalHeadingMonitor(args.port)
    
    print("=" * 60)
    print("    FINAL MAGNETIC HEADING MONITOR")
    print("    Position 40 = Magnetic Heading (Confirmed)")
    print("=" * 60)
    
    if args.test:
        monitor.test_heading()
    else:
        monitor.monitor_heading()

if __name__ == "__main__":
    main()