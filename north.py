#!/usr/bin/env python3
"""
IMU560 North Finder - Fixed Version
Determines how much the device is off from true north
"""
import sys
sys.path.append('/home/pi/Desktop/master/imu560d')

import time
import math
import sys
from imu560 import IMU560, OutputMask

class NorthFinder:
    """Find and display deviation from true north"""
    
    def __init__(self, config_file: str = "config.json"):
        self.imu = IMU560(config_file)
        self.magnetic_declination = 0.0
        self.running = False
        self.continuous_data = None
    
    def connect(self):
        """Connect to IMU and setup for query mode"""
        if not self.imu.connect():
            print("❌ Failed to connect to IMU560")
            return False
        
        print("✅ Connected to IMU560")
        
        # First, stop any continuous mode and clear buffer
        print("🔄 Setting up IMU for query mode...")
        try:
            # Stop continuous mode
            self.imu.set_continuous_mode(False, 1)
            time.sleep(0.5)  # Wait for mode change
            
            # Clear any pending data in buffer
            if self.imu.serial_conn.in_waiting > 0:
                self.imu.serial_conn.reset_input_buffer()
                time.sleep(0.2)
            
            # Set output mask to include magnetometer data
            mask = (OutputMask.EULER | OutputMask.MAGNETOMETERS | 
                   OutputMask.MAG_HEADING | OutputMask.TEMPERATURES)
            if self.imu.set_output_mask(mask):
                print("✅ Output mask configured")
            else:
                print("⚠️ Warning: Could not set output mask")
            
        except Exception as e:
            print(f"⚠️ Setup warning: {e}")
        
        # Try to get magnetic declination (might not be supported)
        try:
            self.magnetic_declination = self.imu.get_magnetic_declination()
            if self.magnetic_declination is not None:
                print(f"📍 Magnetic Declination: {self.magnetic_declination:.2f}°")
            else:
                print("⚠️ Warning: Could not read magnetic declination, using 0°")
                self.magnetic_declination = 0.0
        except Exception:
            print("⚠️ Warning: Magnetic declination not supported, using 0°")
            self.magnetic_declination = 0.0
        
        return True
    
    def calculate_north_deviation(self, magnetic_heading):
        """
        Calculate deviation from true north
        
        Args:
            magnetic_heading: Magnetic heading in degrees (0-360)
            
        Returns:
            tuple: (true_heading, deviation_from_north, direction)
        """
        # Convert magnetic heading to true heading
        true_heading = magnetic_heading + self.magnetic_declination
        
        # Normalize to 0-360 degrees
        true_heading = true_heading % 360
        
        # Calculate deviation from north (0°)
        if true_heading <= 180:
            deviation = true_heading
            direction = "East" if deviation > 0 else ""
        else:
            deviation = 360 - true_heading
            direction = "West"
        
        return true_heading, deviation, direction
    
    def get_compass_direction(self, heading):
        """Convert heading to compass direction"""
        directions = [
            "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
            "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"
        ]
        
        # Each direction covers 22.5 degrees
        index = int((heading + 11.25) / 22.5) % 16
        return directions[index]
    
    def single_reading_with_retry(self, max_retries=3):
        """Get sensor data with retry logic"""
        for attempt in range(max_retries):
            try:
                # Clear buffer first
                if self.imu.serial_conn.in_waiting > 0:
                    self.imu.serial_conn.reset_input_buffer()
                    time.sleep(0.1)
                
                # Get sensor data
                data = self.imu.get_sensor_data()
                if data is not None:
                    return data
                
            except Exception as e:
                print(f"Attempt {attempt + 1} failed: {e}")
                if attempt < max_retries - 1:
                    time.sleep(0.2)
        
        return None
    
    def single_reading(self):
        """Get a single north deviation reading"""
        data = self.single_reading_with_retry()
        if data is None:
            print("❌ Failed to get sensor data after retries")
            return None
        
        # Use magnetic heading if available, otherwise calculate from mag field
        if hasattr(data, 'mag_heading') and data.mag_heading != 0:
            magnetic_heading = data.mag_heading
        else:
            # Calculate heading from magnetometer X and Y components
            if data.mag_x != 0 or data.mag_y != 0:
                magnetic_heading = math.degrees(math.atan2(data.mag_y, data.mag_x))
                if magnetic_heading < 0:
                    magnetic_heading += 360
            else:
                print("❌ No valid magnetic heading data")
                return None
        
        # Calculate north deviation
        true_heading, deviation, direction = self.calculate_north_deviation(magnetic_heading)
        compass_dir = self.get_compass_direction(true_heading)
        
        # Calculate magnetic field strength
        mag_strength = math.sqrt(data.mag_x**2 + data.mag_y**2 + data.mag_z**2)
        
        return {
            'magnetic_heading': magnetic_heading,
            'true_heading': true_heading,
            'deviation': deviation,
            'direction': direction,
            'compass_direction': compass_dir,
            'magnetic_field': mag_strength,
            'mag_x': data.mag_x,
            'mag_y': data.mag_y,
            'mag_z': data.mag_z,
            'roll': getattr(data, 'roll', 0),
            'pitch': getattr(data, 'pitch', 0),
            'yaw': getattr(data, 'yaw', 0)
        }
    
    def continuous_monitoring(self, update_interval=0.5):
        """Continuously monitor north deviation"""
        print("\n" + "="*60)
        print("🧭 CONTINUOUS NORTH MONITORING")
        print("="*60)
        print("Press Ctrl+C to stop")
        print()
        
        self.running = True
        error_count = 0
        
        try:
            while self.running:
                reading = self.single_reading()
                if reading:
                    error_count = 0  # Reset error count on success
                    
                    # Clear line and show updated reading
                    print(f"\r{' '*100}", end='')  # Clear line
                    
                    if reading['deviation'] < 5:
                        status = "🎯 POINTING NORTH!"
                    elif reading['deviation'] < 15:
                        status = "📍 Close to North"
                    elif reading['deviation'] < 45:
                        status = "🔄 Moderate deviation"
                    else:
                        status = "❌ Far from North"
                    
                    direction_text = f" {reading['direction']}" if reading['direction'] else ""
                    
                    print(f"\r{status} | "
                          f"True: {reading['true_heading']:.1f}° ({reading['compass_direction']}) | "
                          f"Off North: {reading['deviation']:.1f}°{direction_text} | "
                          f"Mag: {reading['magnetic_field']:.0f}µT", 
                          end='', flush=True)
                else:
                    error_count += 1
                    if error_count > 5:
                        print(f"\n❌ Too many consecutive errors, stopping...")
                        break
                
                time.sleep(update_interval)
                
        except KeyboardInterrupt:
            self.running = False
            print("\n\n🛑 Monitoring stopped")
    
    def detailed_analysis(self):
        """Perform detailed north analysis"""
        print("\n" + "="*60)
        print("🔍 DETAILED NORTH ANALYSIS")
        print("="*60)
        
        readings = []
        print("Taking 10 readings for analysis...")
        
        # Take multiple readings for averaging
        for i in range(10):
            print(f"Reading {i+1}/10...", end=' ')
            reading = self.single_reading()
            if reading:
                readings.append(reading)
                print(f"✅ {reading['true_heading']:.1f}° ({reading['deviation']:.1f}° off North)")
            else:
                print("❌ Failed")
            time.sleep(0.3)
        
        if not readings:
            print("❌ No valid readings obtained")
            return
        
        # Calculate statistics
        true_headings = [r['true_heading'] for r in readings]
        deviations = [r['deviation'] for r in readings]
        mag_strengths = [r['magnetic_field'] for r in readings]
        
        avg_true_heading = sum(true_headings) / len(true_headings)
        avg_deviation = sum(deviations) / len(deviations)
        avg_mag_strength = sum(mag_strengths) / len(mag_strengths)
        
        # Calculate standard deviation for stability assessment
        heading_std = math.sqrt(sum((h - avg_true_heading)**2 for h in true_headings) / len(true_headings))
        
        print(f"\n📊 ANALYSIS RESULTS:")
        print(f"{'='*40}")
        print(f"Average True Heading: {avg_true_heading:.2f}°")
        print(f"Average Deviation from North: {avg_deviation:.2f}° {readings[0]['direction']}")
        print(f"Compass Direction: {self.get_compass_direction(avg_true_heading)}")
        print(f"Heading Stability: ±{heading_std:.2f}°")
        print(f"Average Magnetic Field: {avg_mag_strength:.1f}µT")
        print(f"Valid Readings: {len(readings)}/10")
        
        # Provide guidance
        print(f"\n🎯 GUIDANCE:")
        if avg_deviation < 2:
            print("🏆 Excellent! Device is very close to true north.")
        elif avg_deviation < 5:
            print("✅ Good! Device is pointing close to north.")
        elif avg_deviation < 15:
            print("📍 Fair. Small adjustment needed to point to north.")
        elif avg_deviation < 45:
            print("🔄 Moderate adjustment needed to point to north.")
        else:
            print("❌ Large adjustment needed to point to north.")
        
        if heading_std > 5:
            print("⚠️ Warning: Heading readings are unstable. Check for magnetic interference.")
        
        if avg_mag_strength < 20:
            print("⚠️ Warning: Weak magnetic field. Consider recalibrating magnetometer.")
        elif avg_mag_strength > 100:
            print("⚠️ Warning: Strong magnetic field. Check for magnetic interference.")
        
        # Show rotation direction
        if avg_deviation > 2:
            if readings[0]['direction'] == "East":
                print(f"🔄 Rotate device {avg_deviation:.1f}° counter-clockwise (left) to point north.")
            elif readings[0]['direction'] == "West":
                print(f"🔄 Rotate device {avg_deviation:.1f}° clockwise (right) to point north.")
    
    def raw_sensor_check(self):
        """Check raw sensor data for troubleshooting"""
        print("\n" + "="*60)
        print("🔧 RAW SENSOR DATA CHECK")
        print("="*60)
        
        reading = self.single_reading()
        if reading:
            print(f"Raw Magnetometer Data:")
            print(f"  X: {reading['mag_x']:.2f} µT")
            print(f"  Y: {reading['mag_y']:.2f} µT")
            print(f"  Z: {reading['mag_z']:.2f} µT")
            print(f"  Total Field: {reading['magnetic_field']:.2f} µT")
            
            print(f"\nCalculated Heading:")
            print(f"  Magnetic: {reading['magnetic_heading']:.2f}°")
            print(f"  True: {reading['true_heading']:.2f}°")
            print(f"  Compass: {reading['compass_direction']}")
            
            if hasattr(reading, 'roll') and reading['roll'] != 0:
                print(f"\nOrientation:")
                print(f"  Roll: {math.degrees(reading['roll']):.2f}°")
                print(f"  Pitch: {math.degrees(reading['pitch']):.2f}°")
                print(f"  Yaw: {math.degrees(reading['yaw']):.2f}°")
            
            print(f"\nMagnetic Declination: {self.magnetic_declination:.2f}°")
        else:
            print("❌ Could not get sensor data")
    
    def interactive_mode(self):
        """Interactive menu for different modes"""
        while True:
            print("\n" + "="*60)
            print("🧭 IMU560 NORTH FINDER")
            print("="*60)
            print("1. Single Reading")
            print("2. Continuous Monitoring")
            print("3. Detailed Analysis") 
            print("4. Raw Sensor Check")
            print("5. Set Magnetic Declination")
            print("6. Exit")
            
            try:
                choice = input("\nSelect option (1-6): ").strip()
                
                if choice == '1':
                    reading = self.single_reading()
                    if reading:
                        print(f"\n🧭 CURRENT HEADING:")
                        print(f"Magnetic Heading: {reading['magnetic_heading']:.1f}°")
                        print(f"True Heading: {reading['true_heading']:.1f}° ({reading['compass_direction']})")
                        direction_text = f" {reading['direction']}" if reading['direction'] else ""
                        print(f"Deviation from North: {reading['deviation']:.1f}°{direction_text}")
                        print(f"Magnetic Field Strength: {reading['magnetic_field']:.1f}µT")
                        
                        if reading['deviation'] < 5:
                            print("🎯 Device is pointing close to North!")
                        elif reading['direction'] == "East":
                            print(f"🔄 Rotate {reading['deviation']:.1f}° counter-clockwise (left) to point North")
                        elif reading['direction'] == "West":
                            print(f"🔄 Rotate {reading['deviation']:.1f}° clockwise (right) to point North")
                    
                elif choice == '2':
                    self.continuous_monitoring()
                    
                elif choice == '3':
                    self.detailed_analysis()
                    
                elif choice == '4':
                    self.raw_sensor_check()
                    
                elif choice == '5':
                    self.set_magnetic_declination()
                    
                elif choice == '6':
                    break
                    
                else:
                    print("Invalid choice. Please select 1-6.")
                    
                if choice != '6':
                    input("\nPress Enter to continue...")
                    
            except KeyboardInterrupt:
                print("\nExiting...")
                break
    
    def set_magnetic_declination(self):
        """Set magnetic declination interactively"""
        print(f"\nCurrent magnetic declination: {self.magnetic_declination:.2f}°")
        print("\nMagnetic declination is the angle between magnetic north and true north.")
        print("Find your local magnetic declination at:")
        print("• https://www.magnetic-declination.com/")
        print("• https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml")
        
        try:
            new_declination = input("\nEnter magnetic declination in degrees (or 'skip'): ").strip()
            if new_declination.lower() != 'skip':
                declination = float(new_declination)
                if -180 <= declination <= 180:
                    # Try to set via IMU command, fallback to local setting
                    try:
                        if self.imu.set_magnetic_declination(declination):
                            self.imu.save_settings()
                            print(f"✅ Magnetic declination set to {declination:.2f}° and saved to IMU")
                        else:
                            print(f"⚠️ IMU command failed, using local setting of {declination:.2f}°")
                    except:
                        print(f"⚠️ IMU command not supported, using local setting of {declination:.2f}°")
                    
                    self.magnetic_declination = declination
                    print(f"✅ Using magnetic declination: {declination:.2f}°")
                else:
                    print("❌ Please enter a value between -180 and 180 degrees")
        except ValueError:
            print("❌ Invalid input. Please enter a number.")
    
    def disconnect(self):
        """Disconnect from IMU"""
        self.running = False
        self.imu.disconnect()

def main():
    """Main function"""
    print("IMU560 North Finder Utility - Fixed Version")
    print("==========================================")
    
    # Handle command line arguments
    config_file = "config.json"
    if len(sys.argv) > 1:
        config_file = sys.argv[1]
        print(f"Using config file: {config_file}")
    
    finder = NorthFinder(config_file)
    
    try:
        if finder.connect():
            finder.interactive_mode()
        else:
            print("Failed to connect to IMU560")
            print("Check your configuration and connections.")
    
    except KeyboardInterrupt:
        print("\nExiting...")
    
    finally:
        finder.disconnect()

if __name__ == "__main__":
    main()