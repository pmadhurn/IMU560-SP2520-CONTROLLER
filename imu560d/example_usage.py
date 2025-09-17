#!/usr/bin/env python3
"""
IMU560 Usage Examples
Demonstrates various ways to use the IMU560 controller
"""

import time
import math
from imu560 import IMU560, OutputMask, DEFAULT_OUTPUT_MASK

def example_basic_usage():
    """Basic IMU usage example"""
    print("="*50)
    print("BASIC USAGE EXAMPLE")
    print("="*50)
    
    # Using context manager for automatic connection cleanup
    with IMU560() as imu:
        if not imu.connect():
            print("Failed to connect to IMU")
            return
        
        print("✓ Connected to IMU560")
        
        # Get single sensor reading
        sensor_data = imu.get_sensor_data()
        if sensor_data:
            print("\n📊 Current Sensor Readings:")
            print(f"Attitude (Roll, Pitch, Yaw): {sensor_data.roll:.3f}, {sensor_data.pitch:.3f}, {sensor_data.yaw:.3f} rad")
            print(f"Attitude (degrees): {math.degrees(sensor_data.roll):.1f}°, {math.degrees(sensor_data.pitch):.1f}°, {math.degrees(sensor_data.yaw):.1f}°")
            print(f"Acceleration: ({sensor_data.accel_x:.3f}, {sensor_data.accel_y:.3f}, {sensor_data.accel_z:.3f}) m/s²")
            print(f"Angular Rate: ({sensor_data.gyro_x:.3f}, {sensor_data.gyro_y:.3f}, {sensor_data.gyro_z:.3f}) rad/s")
            print(f"Magnetic Field: ({sensor_data.mag_x:.1f}, {sensor_data.mag_y:.1f}, {sensor_data.mag_z:.1f}) µT")
            print(f"Temperature: {sensor_data.temp_0:.1f}°C, {sensor_data.temp_1:.1f}°C")
            print(f"Magnetic Heading: {sensor_data.mag_heading:.1f}°")
        else:
            print("Failed to get sensor data")

def example_continuous_mode():
    """Continuous data output example"""
    print("="*50)
    print("CONTINUOUS MODE EXAMPLE")
    print("="*50)
    
    data_count = 0
    
    def data_callback(sensor_data):
        """Callback function for continuous data"""
        nonlocal data_count
        data_count += 1
        
        # Print every 10th reading to avoid flooding
        if data_count % 10 == 0:
            print(f"Reading #{data_count}: Roll={math.degrees(sensor_data.roll):.1f}°, "
                  f"Pitch={math.degrees(sensor_data.pitch):.1f}°, "
                  f"Yaw={math.degrees(sensor_data.yaw):.1f}°")
    
    with IMU560() as imu:
        if not imu.connect():
            print("Failed to connect to IMU")
            return
        
        print("✓ Connected to IMU560")
        print("Starting continuous mode at 100Hz for 10 seconds...")
        print("(Displaying every 10th reading)")
        
        # Start continuous mode with callback
        if imu.start_continuous_mode(callback=data_callback, frequency_divider=1):
            try:
                time.sleep(10)  # Run for 10 seconds
                print(f"\nReceived {data_count} data packets in 10 seconds")
            except KeyboardInterrupt:
                print("\nStopped by user")
            finally:
                imu.stop_continuous_mode()
        else:
            print("Failed to start continuous mode")

def example_custom_output_mask():
    """Example using custom output mask"""
    print("="*50)
    print("CUSTOM OUTPUT MASK EXAMPLE")
    print("="*50)
    
    # Define custom mask with only attitude and gyro data
    custom_mask = OutputMask.EULER | OutputMask.GYROSCOPES | OutputMask.TIME_SINCE_RESET
    
    with IMU560() as imu:
        if not imu.connect():
            print("Failed to connect to IMU")
            return
        
        print("✓ Connected to IMU560")
        
        # Get current mask
        current_mask = imu.get_output_mask()
        print(f"Current output mask: 0x{current_mask:08X}")
        
        # Set custom mask
        print(f"Setting custom mask: 0x{custom_mask:08X} (Euler + Gyro + Time)")
        if imu.set_output_mask(custom_mask):
            print("✓ Custom output mask set")
            
            # Get data with custom mask
            sensor_data = imu.get_sensor_data()
            if sensor_data:
                print("\n📊 Custom Data Reading:")
                print(f"Attitude: Roll={math.degrees(sensor_data.roll):.1f}°, "
                      f"Pitch={math.degrees(sensor_data.pitch):.1f}°, "
                      f"Yaw={math.degrees(sensor_data.yaw):.1f}°")
                print(f"Angular Rate: ({sensor_data.gyro_x:.3f}, {sensor_data.gyro_y:.3f}, {sensor_data.gyro_z:.3f}) rad/s")
                print(f"Time since reset: {sensor_data.time_since_reset} ms")
            
            # Restore original mask
            if imu.set_output_mask(current_mask):
                print("✓ Original output mask restored")
                imu.save_settings()
        else:
            print("✗ Failed to set custom output mask")

def example_configuration():
    """Example showing configuration operations"""
    print("="*50)
    print("CONFIGURATION EXAMPLE")
    print("="*50)
    
    with IMU560() as imu:
        if not imu.connect():
            print("Failed to connect to IMU")
            return
        
        print("✓ Connected to IMU560")
        
        # Get current configuration
        print("\n📋 Current Configuration:")
        
        mask = imu.get_output_mask()
        print(f"Output Mask: 0x{mask:08X}")
        
        baudrate = imu.get_baudrate()
        print(f"Baudrate: {baudrate}")
        
        continuous_mode, divider = imu.get_continuous_mode()
        freq_map = {1: 100, 2: 50, 3: 35, 4: 25, 5: 20, 6: 15, 10: 10, 20: 5, 100: 1}
        freq = freq_map.get(divider, f"1/{divider}")
        print(f"Continuous Mode: {'Enabled' if continuous_mode else 'Disabled'}, Frequency: {freq}Hz")
        
        gravity = imu.get_gravity_magnitude()
        print(f"Gravity Magnitude: {gravity} m/s²")
        
        declination = imu.get_magnetic_declination()
        print(f"Magnetic Declination: {declination}°")

def example_specific_sensors():
    """Example reading specific sensor data"""
    print("="*50)
    print("SPECIFIC SENSORS EXAMPLE")
    print("="*50)
    
    with IMU560() as imu:
        if not imu.connect():
            print("Failed to connect to IMU")
            return
        
        print("✓ Connected to IMU560")
        
        # Read only accelerometer and gyro
        accel_gyro_mask = OutputMask.ACCELEROMETERS | OutputMask.GYROSCOPES
        sensor_data = imu.get_sensor_data(accel_gyro_mask)
        
        if sensor_data:
            print("\n🔄 Motion Sensors Only:")
            print(f"Acceleration: X={sensor_data.accel_x:.3f}, Y={sensor_data.accel_y:.3f}, Z={sensor_data.accel_z:.3f} m/s²")
            print(f"Angular Rate: X={sensor_data.gyro_x:.3f}, Y={sensor_data.gyro_y:.3f}, Z={sensor_data.gyro_z:.3f} rad/s")
            
            # Calculate total acceleration
            total_accel = math.sqrt(sensor_data.accel_x**2 + sensor_data.accel_y**2 + sensor_data.accel_z**2)
            print(f"Total Acceleration: {total_accel:.3f} m/s²")
        
        # Read only magnetometer
        mag_mask = OutputMask.MAGNETOMETERS | OutputMask.MAG_HEADING
        sensor_data = imu.get_sensor_data(mag_mask)
        
        if sensor_data:
            print("\n🧭 Magnetometer Only:")
            print(f"Magnetic Field: X={sensor_data.mag_x:.1f}, Y={sensor_data.mag_y:.1f}, Z={sensor_data.mag_z:.1f} µT")
            print(f"Magnetic Heading: {sensor_data.mag_heading:.1f}°")
            
            # Calculate total magnetic field
            total_mag = math.sqrt(sensor_data.mag_x**2 + sensor_data.mag_y**2 + sensor_data.mag_z**2)
            print(f"Total Magnetic Field: {total_mag:.1f} µT")

def example_gps_data():
    """Example reading GPS and navigation data"""
    print("="*50)
    print("GPS AND NAVIGATION EXAMPLE")
    print("="*50)
    
    gps_mask = (OutputMask.GPS_INFO | OutputMask.POSITION | 
                OutputMask.VELOCITY | OutputMask.UTC_TIME_REFERENCE |
                OutputMask.BARO_ALTITUDE | OutputMask.BARO_PRESSURE)
    
    with IMU560() as imu:
        if not imu.connect():
            print("Failed to connect to IMU")
            return
        
        print("✓ Connected to IMU560")
        print("Reading GPS and navigation data...")
        
        sensor_data = imu.get_sensor_data(gps_mask)
        
        if sensor_data:
            print("\n🛰️ GPS Information:")
            print(f"GPS Time of Week: {sensor_data.gps_itow} ms")
            print(f"GPS Flags: 0x{sensor_data.gps_flags:02X}")
            print(f"Satellites: {sensor_data.gps_num_sv}")
            
            # Parse GPS flags
            gps_fix = sensor_data.gps_flags & 0x03
            fix_types = ["No Fix", "Time Only", "2D Fix", "3D Fix"]
            print(f"GPS Fix Type: {fix_types[gps_fix]}")
            
            print(f"\n📍 Position (WGS84):")
            print(f"Latitude: {sensor_data.latitude:.8f}°")
            print(f"Longitude: {sensor_data.longitude:.8f}°")
            print(f"Altitude: {sensor_data.altitude:.2f} m")
            
            print(f"\n🏃 Velocity:")
            print(f"North: {sensor_data.vel_north:.3f} m/s")
            print(f"East: {sensor_data.vel_east:.3f} m/s")
            print(f"Down: {sensor_data.vel_down:.3f} m/s")
            
            # Calculate ground speed
            ground_speed = math.sqrt(sensor_data.vel_north**2 + sensor_data.vel_east**2)
            print(f"Ground Speed: {ground_speed:.3f} m/s ({ground_speed*3.6:.1f} km/h)")
            
            print(f"\n⏰ UTC Time:")
            print(f"{sensor_data.utc_year}-{sensor_data.utc_month:02d}-{sensor_data.utc_day:02d} "
                  f"{sensor_data.utc_hour:02d}:{sensor_data.utc_minute:02d}:{sensor_data.utc_second:02d}")
            
            print(f"\n🌡️ Barometric Data:")
            print(f"Altitude: {sensor_data.baro_altitude/100:.2f} m")  # Convert cm to m
            print(f"Pressure: {sensor_data.baro_pressure} Pa ({sensor_data.baro_pressure/100:.2f} hPa)")

def main():
    """Main function with example menu"""
    print("IMU560 Usage Examples")
    print("====================")
    
    examples = {
        '1': ("Basic Usage", example_basic_usage),
        '2': ("Continuous Mode", example_continuous_mode),
        '3': ("Custom Output Mask", example_custom_output_mask),
        '4': ("Configuration Info", example_configuration),
        '5': ("Specific Sensors", example_specific_sensors),
        '6': ("GPS & Navigation", example_gps_data),
    }
    
    while True:
        print("\n" + "="*40)
        print("SELECT EXAMPLE TO RUN")
        print("="*40)
        
        for key, (name, _) in examples.items():
            print(f"{key}. {name}")
        print("7. Run all examples")
        print("8. Exit")
        
        try:
            choice = input("\nSelect option (1-8): ").strip()
            
            if choice in examples:
                print(f"\nRunning: {examples[choice][0]}")
                examples[choice][1]()
                input("\nPress Enter to continue...")
                
            elif choice == '7':
                print("\nRunning all examples...")
                for name, func in examples.values():
                    print(f"\n{'='*20} {name} {'='*20}")
                    try:
                        func()
                    except Exception as e:
                        print(f"Error in {name}: {e}")
                    time.sleep(2)  # Brief pause between examples
                print("\nAll examples completed!")
                input("\nPress Enter to continue...")
                
            elif choice == '8':
                print("Exiting examples")
                break
                
            else:
                print("Invalid choice. Please select 1-8.")
                
        except KeyboardInterrupt:
            print("\nExiting examples")
            break
        except Exception as e:
            print(f"Error: {e}")

if __name__ == "__main__":
    main()