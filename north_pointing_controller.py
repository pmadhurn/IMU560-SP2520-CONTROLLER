#!/usr/bin/env python3
"""
SP2520 North Pointing Controller
Automatically keeps SP2520 PTZ camera pointing toward magnetic north using IMU560
"""
import sys
sys.path.append('/home/pi/Desktop/master/imu560d')

import time
import math
import threading
from imu560 import IMU560, OutputMask

# Import your SP2520Controller (assuming it's in the same directory)
from sp2520_controller import SP2520Controller

class NorthPointingController:
    """Controller that keeps SP2520 pointing north using IMU560"""
    
    def __init__(self, sp2520_port='/dev/ttyUSB1', imu_config="config.json"):
        # Initialize both devices
        self.sp2520 = SP2520Controller(port=sp2520_port)
        self.imu = IMU560(imu_config)
        
        # Control parameters
        self.target_heading = 0.0  # North
        self.heading_tolerance = 2.0  # degrees
        self.max_correction_angle = 5.0  # Maximum correction per step
        self.update_interval = 1.0  # seconds between corrections
        self.magnetic_declination = 0.0
        
        # State variables
        self.running = False
        self.current_heading = None
        self.control_thread = None
        self.last_correction_time = 0
        
        print("🔧 North Pointing Controller Initialized")
    
    def connect_devices(self):
        """Connect to both SP2520 and IMU560"""
        print("🔄 Connecting to devices...")
        
        # Connect to IMU560
        if not self.imu.connect():
            print("❌ Failed to connect to IMU560")
            return False
        print("✅ Connected to IMU560")
        
        # Setup IMU for magnetometer readings
        try:
            self.imu.set_continuous_mode(False, 1)
            time.sleep(0.5)
            
            mask = (OutputMask.EULER | OutputMask.MAGNETOMETERS | 
                   OutputMask.MAG_HEADING)
            self.imu.set_output_mask(mask)
            
            # Get magnetic declination
            try:
                self.magnetic_declination = self.imu.get_magnetic_declination() or 0.0
                print(f"📍 Magnetic Declination: {self.magnetic_declination:.2f}°")
            except:
                print("⚠️ Using magnetic declination: 0.0°")
                
        except Exception as e:
            print(f"⚠️ IMU setup warning: {e}")
        
        print("✅ Both devices connected and configured")
        return True
    
    def get_current_heading(self):
        """Get current magnetic heading from IMU560"""
        try:
            # Clear buffer
            if self.imu.serial_conn.in_waiting > 0:
                self.imu.serial_conn.reset_input_buffer()
                time.sleep(0.1)
            
            # Get sensor data
            data = self.imu.get_sensor_data()
            if data is None:
                return None
            
            # Calculate magnetic heading
            if hasattr(data, 'mag_heading') and data.mag_heading != 0:
                magnetic_heading = data.mag_heading
            else:
                # Calculate from magnetometer components
                if data.mag_x != 0 or data.mag_y != 0:
                    magnetic_heading = math.degrees(math.atan2(data.mag_y, data.mag_x))
                    if magnetic_heading < 0:
                        magnetic_heading += 360
                else:
                    return None
            
            # Convert to true heading
            true_heading = (magnetic_heading + self.magnetic_declination) % 360
            return true_heading
            
        except Exception as e:
            print(f"❌ Error reading heading: {e}")
            return None
    
    def calculate_correction(self, current_heading):
        """
        Calculate how much to rotate to point north
        
        Args:
            current_heading: Current heading in degrees (0-360)
            
        Returns:
            tuple: (correction_angle, direction)
                correction_angle: degrees to rotate (positive value)
                direction: 'left' or 'right' or 'none'
        """
        if current_heading is None:
            return 0, 'none'
        
        # Calculate shortest rotation to north (0°)
        error = current_heading - self.target_heading
        
        # Normalize error to [-180, 180]
        while error > 180:
            error -= 360
        while error < -180:
            error += 360
        
        # Check if within tolerance
        if abs(error) <= self.heading_tolerance:
            return 0, 'none'
        
        # Limit correction angle
        correction_angle = min(abs(error), self.max_correction_angle)
        
        # Determine direction
        if error > 0:
            direction = 'left'  # Rotate counter-clockwise
        else:
            direction = 'right'  # Rotate clockwise
        
        return correction_angle, direction
    
    def apply_correction(self, correction_angle, direction):
        """Apply correction to SP2520"""
        if direction == 'none' or correction_angle == 0:
            return True
        
        # Calculate movement time based on correction angle
        # Assuming pan speed of 0x20 gives roughly X degrees per second
        # You may need to calibrate this based on your SP2520 settings
        movement_time = correction_angle / 10.0  # Rough estimate
        movement_time = max(0.1, min(movement_time, 2.0))  # Limit between 0.1-2 seconds
        
        try:
            print(f"🔄 Correcting {correction_angle:.1f}° to the {direction}")
            
            # Move in the calculated direction
            if direction == 'left':
                self.sp2520.move_left()
            else:  # direction == 'right'
                self.sp2520.move_right()
            
            # Move for calculated time
            time.sleep(movement_time)
            
            # Stop movement
            self.sp2520.stop()
            
            # Wait for stabilization
            time.sleep(0.5)
            
            return True
            
        except Exception as e:
            print(f"❌ Error applying correction: {e}")
            self.sp2520.stop()  # Ensure it stops on error
            return False
    
    def control_loop(self):
        """Main control loop that runs in background thread"""
        print("🎯 Starting north pointing control loop")
        
        consecutive_errors = 0
        max_errors = 5
        
        while self.running:
            try:
                # Get current heading
                current_heading = self.get_current_heading()
                self.current_heading = current_heading
                
                if current_heading is not None:
                    consecutive_errors = 0  # Reset error count
                    
                    # Calculate required correction
                    correction_angle, direction = self.calculate_correction(current_heading)
                    
                    # Apply correction if needed
                    if direction != 'none':
                        if self.apply_correction(correction_angle, direction):
                            print(f"✅ Correction applied: {correction_angle:.1f}° {direction}")
                        else:
                            print("❌ Failed to apply correction")
                    else:
                        print(f"🎯 On target: {current_heading:.1f}° (within {self.heading_tolerance}°)")
                
                else:
                    consecutive_errors += 1
                    print(f"❌ Could not read heading (error {consecutive_errors}/{max_errors})")
                    
                    if consecutive_errors >= max_errors:
                        print("❌ Too many consecutive errors, stopping control loop")
                        break
                
                # Wait before next update
                time.sleep(self.update_interval)
                
            except Exception as e:
                print(f"❌ Control loop error: {e}")
                consecutive_errors += 1
                if consecutive_errors >= max_errors:
                    break
                time.sleep(1)
        
        print("🛑 North pointing control loop stopped")
    
    def start_north_pointing(self):
        """Start automatic north pointing"""
        if self.running:
            print("⚠️ North pointing already active")
            return False
        
        print("🚀 Starting automatic north pointing...")
        self.running = True
        
        # Start control loop in background thread
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
        
        return True
    
    def stop_north_pointing(self):
        """Stop automatic north pointing"""
        print("🛑 Stopping north pointing...")
        self.running = False
        
        # Stop SP2520 movement
        self.sp2520.stop()
        
        # Wait for control thread to finish
        if self.control_thread and self.control_thread.is_alive():
            self.control_thread.join(timeout=5)
        
        print("✅ North pointing stopped")
    
    def get_status(self):
        """Get current status"""
        return {
            'running': self.running,
            'current_heading': self.current_heading,
            'target_heading': self.target_heading,
            'heading_tolerance': self.heading_tolerance,
            'magnetic_declination': self.magnetic_declination
        }
    
    def set_parameters(self, tolerance=None, max_correction=None, update_interval=None):
        """Update control parameters"""
        if tolerance is not None:
            self.heading_tolerance = max(0.1, min(tolerance, 10.0))
            print(f"✅ Heading tolerance set to {self.heading_tolerance:.1f}°")
        
        if max_correction is not None:
            self.max_correction_angle = max(1.0, min(max_correction, 20.0))
            print(f"✅ Max correction angle set to {self.max_correction_angle:.1f}°")
        
        if update_interval is not None:
            self.update_interval = max(0.5, min(update_interval, 10.0))
            print(f"✅ Update interval set to {self.update_interval:.1f}s")
    
    def manual_point_north(self):
        """Manually point to north once"""
        print("🎯 Manual north pointing...")
        
        # Get current heading
        current_heading = self.get_current_heading()
        if current_heading is None:
            print("❌ Could not read current heading")
            return False
        
        print(f"📍 Current heading: {current_heading:.1f}°")
        
        # Calculate and apply correction
        correction_angle, direction = self.calculate_correction(current_heading)
        
        if direction == 'none':
            print("🎯 Already pointing north!")
            return True
        
        print(f"🔄 Need to rotate {correction_angle:.1f}° {direction}")
        return self.apply_correction(correction_angle, direction)
    
    def interactive_mode(self):
        """Interactive control interface"""
        while True:
            print("\n" + "="*60)
            print("🧭 SP2520 NORTH POINTING CONTROLLER")
            print("="*60)
            print("1. Manual Point North (once)")
            print("2. Start Automatic North Pointing")
            print("3. Stop Automatic North Pointing")
            print("4. Show Status")
            print("5. Set Parameters")
            print("6. Test Current Heading")
            print("7. Exit")
            
            try:
                choice = input("\nSelect option (1-7): ").strip()
                
                if choice == '1':
                    self.manual_point_north()
                
                elif choice == '2':
                    if self.start_north_pointing():
                        print("✅ Automatic north pointing started")
                        print("Press option 3 to stop")
                
                elif choice == '3':
                    self.stop_north_pointing()
                
                elif choice == '4':
                    status = self.get_status()
                    print(f"\n📊 STATUS:")
                    print(f"Running: {'✅ Yes' if status['running'] else '❌ No'}")
                    if status['current_heading'] is not None:
                        print(f"Current Heading: {status['current_heading']:.1f}°")
                        deviation = abs(status['current_heading'] - status['target_heading'])
                        if deviation > 180:
                            deviation = 360 - deviation
                        print(f"Deviation from North: {deviation:.1f}°")
                    print(f"Tolerance: {status['heading_tolerance']:.1f}°")
                    print(f"Magnetic Declination: {status['magnetic_declination']:.1f}°")
                
                elif choice == '5':
                    print("\nCurrent Parameters:")
                    print(f"Heading Tolerance: {self.heading_tolerance:.1f}°")
                    print(f"Max Correction: {self.max_correction_angle:.1f}°")
                    print(f"Update Interval: {self.update_interval:.1f}s")
                    
                    try:
                        tol = input("New tolerance (degrees, Enter to skip): ").strip()
                        max_corr = input("New max correction (degrees, Enter to skip): ").strip()
                        interval = input("New update interval (seconds, Enter to skip): ").strip()
                        
                        self.set_parameters(
                            tolerance=float(tol) if tol else None,
                            max_correction=float(max_corr) if max_corr else None,
                            update_interval=float(interval) if interval else None
                        )
                    except ValueError:
                        print("❌ Invalid input")
                
                elif choice == '6':
                    heading = self.get_current_heading()
                    if heading is not None:
                        print(f"📍 Current Heading: {heading:.1f}°")
                        deviation = abs(heading)
                        if deviation > 180:
                            deviation = 360 - deviation
                        print(f"Deviation from North: {deviation:.1f}°")
                    else:
                        print("❌ Could not read heading")
                
                elif choice == '7':
                    break
                
                else:
                    print("Invalid choice. Please select 1-7.")
                
                if choice != '7':
                    input("\nPress Enter to continue...")
                
            except KeyboardInterrupt:
                print("\nExiting...")
                break
    
    def disconnect(self):
        """Clean up and disconnect"""
        self.stop_north_pointing()
        self.sp2520.close()
        self.imu.disconnect()
        print("✅ Disconnected from all devices")

def main():
    """Main function"""
    print("SP2520 North Pointing Controller")
    print("================================")
    
    # You may need to adjust these ports
    controller = NorthPointingController(
        sp2520_port='/dev/ttyUSB1',  # Adjust as needed
        imu_config="config.json"
    )
    
    try:
        if controller.connect_devices():
            controller.interactive_mode()
        else:
            print("Failed to connect to devices")
    
    except KeyboardInterrupt:
        print("\nExiting...")
    
    finally:
        controller.disconnect()

if __name__ == "__main__":
    main()