#!/usr/bin/env python3
"""
Simple North Finder Script
The simplest way to point SP2520 camera toward magnetic north using IMU560
"""
import sys
import os
import time
import math

# Add paths
sys.path.append('imu560d')
sys.path.append('sp250')

from imu560 import IMU560, OutputMask
from sp2520_controller import SP2520Controller

def find_north():
    """Simple function to find and point toward north"""
    print("🧭 Simple North Finder Starting...")
    
    # Initialize devices
    imu = IMU560("imu560d/config.json")
    ptz = SP2520Controller(port='/dev/ttyUSB1')  # Adjust port as needed
    
    try:
        # Connect to IMU
        print("📡 Connecting to IMU560...")
        if not imu.connect():
            print("❌ Failed to connect to IMU560")
            return False
        
        # Configure IMU for magnetometer readings
        print("⚙️ Configuring IMU...")
        imu.set_continuous_mode(False, 1)
        time.sleep(0.5)
        
        # Set output mask for magnetometer and heading
        mask = OutputMask.MAGNETOMETERS | OutputMask.MAG_HEADING | OutputMask.EULER
        imu.set_output_mask(mask)
        time.sleep(0.5)
        
        # Get magnetic declination
        try:
            declination = imu.get_magnetic_declination() or 0.0
            print(f"📍 Magnetic Declination: {declination:.1f}°")
        except:
            declination = 0.0
            print("📍 Using default declination: 0.0°")
        
        print("🎯 Finding current heading...")
        
        # Get current heading
        for attempt in range(5):
            data = imu.get_sensor_data()
            if data:
                # Calculate magnetic heading
                if data.mag_heading != 0:
                    magnetic_heading = data.mag_heading
                elif data.mag_x != 0 or data.mag_y != 0:
                    magnetic_heading = math.degrees(math.atan2(data.mag_y, data.mag_x))
                    if magnetic_heading < 0:
                        magnetic_heading += 360
                else:
                    print(f"⏳ Attempt {attempt + 1}: No magnetometer data, retrying...")
                    time.sleep(1)
                    continue
                
                # Calculate true heading
                true_heading = (magnetic_heading + declination) % 360
                
                print(f"📊 Current Heading: {true_heading:.1f}°")
                print(f"📊 Magnetic Heading: {magnetic_heading:.1f}°")
                
                # Calculate how much to rotate to point north (0°)
                error = true_heading
                if error > 180:
                    error = error - 360
                
                print(f"🔄 Need to rotate: {abs(error):.1f}° {'left' if error > 0 else 'right'}")
                
                if abs(error) <= 2.0:
                    print("🎯 Already pointing north!")
                    return True
                
                # Calculate movement time (rough estimate: 10 degrees per second)
                movement_time = abs(error) / 10.0
                movement_time = max(0.1, min(movement_time, 5.0))  # Limit between 0.1-5 seconds
                
                print(f"⏱️ Moving for {movement_time:.1f} seconds...")
                
                # Move toward north
                if error > 0:
                    ptz.move_left()
                else:
                    ptz.move_right()
                
                time.sleep(movement_time)
                ptz.stop()
                
                print("✅ Movement complete!")
                
                # Verify final position
                time.sleep(1)
                final_data = imu.get_sensor_data()
                if final_data:
                    if final_data.mag_heading != 0:
                        final_magnetic = final_data.mag_heading
                    else:
                        final_magnetic = math.degrees(math.atan2(final_data.mag_y, final_data.mag_x))
                        if final_magnetic < 0:
                            final_magnetic += 360
                    
                    final_true = (final_magnetic + declination) % 360
                    final_error = final_true if final_true <= 180 else final_true - 360
                    
                    print(f"🎯 Final Heading: {final_true:.1f}°")
                    print(f"📏 Final Error: {abs(final_error):.1f}°")
                    
                    if abs(final_error) <= 5.0:
                        print("🎉 Successfully pointed toward north!")
                        return True
                    else:
                        print("⚠️ Still not accurate enough, but close")
                        return True
                
                return True
            
            print(f"⏳ Attempt {attempt + 1}: No sensor data, retrying...")
            time.sleep(1)
        
        print("❌ Failed to get sensor data after 5 attempts")
        return False
        
    except Exception as e:
        print(f"❌ Error: {e}")
        return False
    
    finally:
        # Clean up
        ptz.stop()
        ptz.close()
        imu.disconnect()
        print("🔌 Disconnected from devices")

if __name__ == "__main__":
    print("=" * 50)
    print("🧭 SIMPLE NORTH FINDER")
    print("=" * 50)
    print("This script will:")
    print("1. Connect to IMU560 for heading data")
    print("2. Calculate current direction")
    print("3. Rotate SP2520 camera toward north")
    print("4. Verify final position")
    print()
    
    input("Press Enter to start...")
    
    success = find_north()
    
    if success:
        print("\n🎉 North finding completed!")
    else:
        print("\n❌ North finding failed!")
    
    print("\nDone.")