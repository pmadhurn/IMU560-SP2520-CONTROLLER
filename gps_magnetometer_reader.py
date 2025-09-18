#!/usr/bin/env python3
"""
GPS and Magnetometer Data Reader for IMU560
Provides accurate GPS coordinates and magnetometer readings
"""
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), 'imu560d'))

import time
import math
import json
from datetime import datetime
from imu560 import IMU560, OutputMask

class GPSMagnetometerReader:
    """Reader for GPS coordinates and magnetometer data from IMU560"""
    
    def __init__(self, config_file="imu560d/config.json"):
        """Initialize the GPS and Magnetometer reader"""
        self.imu = IMU560(config_file)
        self.connected = False
        self.magnetic_declination = 0.0
        
        print("🛰️ GPS & Magnetometer Reader Initialized")
    
    def connect(self):
        """Connect to IMU560 and configure for GPS and magnetometer readings"""
        print("🔄 Connecting to IMU560...")
        
        if not self.imu.connect():
            print("❌ Failed to connect to IMU560")
            return False
        
        print("✅ Connected to IMU560")
        
        try:
            # Configure IMU for GPS and magnetometer data
            self.imu.set_continuous_mode(False, 1)
            time.sleep(0.5)
            
            # Set output mask for GPS, magnetometer, and related data
            mask = (OutputMask.GPS_POSITION | OutputMask.GPS_NAVIGATION | 
                   OutputMask.GPS_ACCURACY | OutputMask.GPS_INFO |
                   OutputMask.MAGNETOMETERS | OutputMask.MAG_HEADING |
                   OutputMask.EULER | OutputMask.UTC_TIME_REFERENCE |
                   OutputMask.VELOCITY | OutputMask.BARO_ALTITUDE)
            
            self.imu.set_output_mask(mask)
            time.sleep(0.5)
            
            # Get magnetic declination
            try:
                self.magnetic_declination = self.imu.get_magnetic_declination() or 0.0
                print(f"📍 Magnetic Declination: {self.magnetic_declination:.2f}°")
            except:
                print("⚠️ Using default magnetic declination: 0.0°")
            
            self.connected = True
            print("✅ IMU560 configured for GPS and magnetometer readings")
            return True
            
        except Exception as e:
            print(f"❌ Error configuring IMU560: {e}")
            return False
    
    def get_gps_coordinates(self):
        """
        Get accurate GPS coordinates
        
        Returns:
            dict: GPS data with latitude, longitude, altitude, accuracy info
        """
        if not self.connected:
            print("❌ Not connected to IMU560")
            return None
        
        try:
            # Clear buffer for fresh data
            if self.imu.serial_conn.in_waiting > 0:
                self.imu.serial_conn.reset_input_buffer()
                time.sleep(0.1)
            
            # Get sensor data
            data = self.imu.get_sensor_data()
            if data is None:
                return None
            
            # Extract GPS information
            gps_data = {
                'latitude': data.latitude,
                'longitude': data.longitude,
                'altitude': data.altitude,
                'velocity': {
                    'north': data.vel_north,
                    'east': data.vel_east,
                    'down': data.vel_down
                },
                'gps_info': {
                    'itow': data.gps_itow,
                    'flags': data.gps_flags,
                    'num_satellites': data.gps_num_sv
                },
                'utc_time': {
                    'year': data.utc_year,
                    'month': data.utc_month,
                    'day': data.utc_day,
                    'hour': data.utc_hour,
                    'minute': data.utc_minute,
                    'second': data.utc_second
                },
                'barometric_altitude': data.baro_altitude / 100.0,  # Convert cm to meters
                'timestamp': time.time()
            }
            
            return gps_data
            
        except Exception as e:
            print(f"❌ Error reading GPS data: {e}")
            return None
    
    def get_magnetometer_readings(self):
        """
        Get current magnetometer readings and calculated heading
        
        Returns:
            dict: Magnetometer data with raw values and calculated heading
        """
        if not self.connected:
            print("❌ Not connected to IMU560")
            return None
        
        try:
            # Clear buffer for fresh data
            if self.imu.serial_conn.in_waiting > 0:
                self.imu.serial_conn.reset_input_buffer()
                time.sleep(0.1)
            
            # Get sensor data
            data = self.imu.get_sensor_data()
            if data is None:
                return None
            
            # Calculate magnetic heading from magnetometer components
            magnetic_heading = None
            if data.mag_heading != 0:
                magnetic_heading = data.mag_heading
            elif data.mag_x != 0 or data.mag_y != 0:
                magnetic_heading = math.degrees(math.atan2(data.mag_y, data.mag_x))
                if magnetic_heading < 0:
                    magnetic_heading += 360
            
            # Calculate true heading (compensated for magnetic declination)
            true_heading = None
            if magnetic_heading is not None:
                true_heading = (magnetic_heading + self.magnetic_declination) % 360
            
            # Calculate magnetic field strength
            mag_strength = math.sqrt(data.mag_x**2 + data.mag_y**2 + data.mag_z**2)
            
            magnetometer_data = {
                'raw_values': {
                    'mag_x': data.mag_x,
                    'mag_y': data.mag_y,
                    'mag_z': data.mag_z
                },
                'magnetic_heading': magnetic_heading,
                'true_heading': true_heading,
                'magnetic_declination': self.magnetic_declination,
                'field_strength': mag_strength,
                'euler_angles': {
                    'roll': math.degrees(data.roll),
                    'pitch': math.degrees(data.pitch),
                    'yaw': math.degrees(data.yaw)
                },
                'timestamp': time.time()
            }
            
            return magnetometer_data
            
        except Exception as e:
            print(f"❌ Error reading magnetometer data: {e}")
            return None
    
    def get_combined_data(self):
        """
        Get both GPS and magnetometer data in a single call
        
        Returns:
            dict: Combined GPS and magnetometer data
        """
        if not self.connected:
            print("❌ Not connected to IMU560")
            return None
        
        try:
            # Clear buffer for fresh data
            if self.imu.serial_conn.in_waiting > 0:
                self.imu.serial_conn.reset_input_buffer()
                time.sleep(0.1)
            
            # Get sensor data
            data = self.imu.get_sensor_data()
            if data is None:
                return None
            
            # Calculate magnetic heading
            magnetic_heading = None
            if data.mag_heading != 0:
                magnetic_heading = data.mag_heading
            elif data.mag_x != 0 or data.mag_y != 0:
                magnetic_heading = math.degrees(math.atan2(data.mag_y, data.mag_x))
                if magnetic_heading < 0:
                    magnetic_heading += 360
            
            true_heading = None
            if magnetic_heading is not None:
                true_heading = (magnetic_heading + self.magnetic_declination) % 360
            
            # Calculate magnetic field strength
            mag_strength = math.sqrt(data.mag_x**2 + data.mag_y**2 + data.mag_z**2)
            
            combined_data = {
                'gps': {
                    'latitude': data.latitude,
                    'longitude': data.longitude,
                    'altitude': data.altitude,
                    'velocity': {
                        'north': data.vel_north,
                        'east': data.vel_east,
                        'down': data.vel_down
                    },
                    'gps_info': {
                        'itow': data.gps_itow,
                        'flags': data.gps_flags,
                        'num_satellites': data.gps_num_sv
                    },
                    'barometric_altitude': data.baro_altitude / 100.0
                },
                'magnetometer': {
                    'raw_values': {
                        'mag_x': data.mag_x,
                        'mag_y': data.mag_y,
                        'mag_z': data.mag_z
                    },
                    'magnetic_heading': magnetic_heading,
                    'true_heading': true_heading,
                    'field_strength': mag_strength
                },
                'attitude': {
                    'roll': math.degrees(data.roll),
                    'pitch': math.degrees(data.pitch),
                    'yaw': math.degrees(data.yaw)
                },
                'utc_time': {
                    'year': data.utc_year,
                    'month': data.utc_month,
                    'day': data.utc_day,
                    'hour': data.utc_hour,
                    'minute': data.utc_minute,
                    'second': data.utc_second
                },
                'magnetic_declination': self.magnetic_declination,
                'timestamp': time.time()
            }
            
            return combined_data
            
        except Exception as e:
            print(f"❌ Error reading combined data: {e}")
            return None
    
    def save_data_to_file(self, data, filename=None):
        """Save data to JSON file"""
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"gps_mag_data_{timestamp}.json"
        
        try:
            with open(filename, 'w') as f:
                json.dump(data, f, indent=2)
            print(f"✅ Data saved to {filename}")
            return True
        except Exception as e:
            print(f"❌ Error saving data: {e}")
            return False
    
    def continuous_monitoring(self, duration=60, interval=1.0, save_to_file=False):
        """
        Continuously monitor GPS and magnetometer data
        
        Args:
            duration: Monitoring duration in seconds
            interval: Time between readings in seconds
            save_to_file: Whether to save data to file
        """
        print(f"🔄 Starting continuous monitoring for {duration} seconds...")
        
        data_log = []
        start_time = time.time()
        
        try:
            while time.time() - start_time < duration:
                data = self.get_combined_data()
                if data:
                    # Display current readings
                    gps = data['gps']
                    mag = data['magnetometer']
                    
                    print(f"\n📍 GPS: {gps['latitude']:.6f}, {gps['longitude']:.6f}, Alt: {gps['altitude']:.1f}m")
                    print(f"🧭 Heading: {mag['true_heading']:.1f}° (Mag: {mag['magnetic_heading']:.1f}°)")
                    print(f"🛰️ Satellites: {gps['gps_info']['num_satellites']}")
                    
                    if save_to_file:
                        data_log.append(data)
                
                time.sleep(interval)
        
        except KeyboardInterrupt:
            print("\n🛑 Monitoring stopped by user")
        
        if save_to_file and data_log:
            self.save_data_to_file(data_log)
        
        print("✅ Monitoring completed")
    
    def disconnect(self):
        """Disconnect from IMU560"""
        if self.connected:
            self.imu.disconnect()
            self.connected = False
            print("✅ Disconnected from IMU560")

def main():
    """Main function for testing"""
    print("GPS & Magnetometer Reader for IMU560")
    print("====================================")
    
    reader = GPSMagnetometerReader()
    
    try:
        if reader.connect():
            while True:
                print("\n" + "="*50)
                print("📡 GPS & MAGNETOMETER READER")
                print("="*50)
                print("1. Get GPS Coordinates")
                print("2. Get Magnetometer Readings")
                print("3. Get Combined Data")
                print("4. Continuous Monitoring (60s)")
                print("5. Save Current Data to File")
                print("6. Exit")
                
                choice = input("\nSelect option (1-6): ").strip()
                
                if choice == '1':
                    gps_data = reader.get_gps_coordinates()
                    if gps_data:
                        print(f"\n📍 GPS Coordinates:")
                        print(f"Latitude: {gps_data['latitude']:.6f}°")
                        print(f"Longitude: {gps_data['longitude']:.6f}°")
                        print(f"Altitude: {gps_data['altitude']:.1f}m")
                        print(f"Barometric Alt: {gps_data['barometric_altitude']:.1f}m")
                        print(f"Satellites: {gps_data['gps_info']['num_satellites']}")
                        print(f"Velocity N/E/D: {gps_data['velocity']['north']:.2f}/{gps_data['velocity']['east']:.2f}/{gps_data['velocity']['down']:.2f} m/s")
                
                elif choice == '2':
                    mag_data = reader.get_magnetometer_readings()
                    if mag_data:
                        print(f"\n🧭 Magnetometer Readings:")
                        print(f"True Heading: {mag_data['true_heading']:.1f}°")
                        print(f"Magnetic Heading: {mag_data['magnetic_heading']:.1f}°")
                        print(f"Field Strength: {mag_data['field_strength']:.1f} µT")
                        print(f"Raw Values - X: {mag_data['raw_values']['mag_x']:.2f}, Y: {mag_data['raw_values']['mag_y']:.2f}, Z: {mag_data['raw_values']['mag_z']:.2f}")
                        print(f"Attitude - Roll: {mag_data['euler_angles']['roll']:.1f}°, Pitch: {mag_data['euler_angles']['pitch']:.1f}°, Yaw: {mag_data['euler_angles']['yaw']:.1f}°")
                
                elif choice == '3':
                    combined_data = reader.get_combined_data()
                    if combined_data:
                        gps = combined_data['gps']
                        mag = combined_data['magnetometer']
                        att = combined_data['attitude']
                        
                        print(f"\n📊 Combined Data:")
                        print(f"📍 GPS: {gps['latitude']:.6f}, {gps['longitude']:.6f}, Alt: {gps['altitude']:.1f}m")
                        print(f"🧭 True Heading: {mag['true_heading']:.1f}° (Magnetic: {mag['magnetic_heading']:.1f}°)")
                        print(f"🛰️ Satellites: {gps['gps_info']['num_satellites']}")
                        print(f"📐 Attitude: Roll {att['roll']:.1f}°, Pitch {att['pitch']:.1f}°, Yaw {att['yaw']:.1f}°")
                        print(f"🧲 Field Strength: {mag['field_strength']:.1f} µT")
                
                elif choice == '4':
                    reader.continuous_monitoring(duration=60, interval=1.0, save_to_file=True)
                
                elif choice == '5':
                    data = reader.get_combined_data()
                    if data:
                        reader.save_data_to_file(data)
                
                elif choice == '6':
                    break
                
                else:
                    print("Invalid choice. Please select 1-6.")
                
                if choice != '6':
                    input("\nPress Enter to continue...")
        
        else:
            print("Failed to connect to IMU560")
    
    except KeyboardInterrupt:
        print("\nExiting...")
    
    finally:
        reader.disconnect()

if __name__ == "__main__":
    main()