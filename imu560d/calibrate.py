#!/usr/bin/env python3
"""
IMU560 Calibration Script
Provides interactive calibration for magnetometer and other sensors
"""

import time
import sys
from imu560 import IMU560, logger

class IMUCalibrator:
    """Interactive IMU calibration utility"""
    
    def __init__(self, config_file: str = "config.json"):
        self.imu = IMU560(config_file)
        self.connected = False
    
    def connect(self):
        """Connect to IMU"""
        if self.imu.connect():
            self.connected = True
            print("✓ Connected to IMU560")
            return True
        else:
            print("✗ Failed to connect to IMU560")
            return False
    
    def disconnect(self):
        """Disconnect from IMU"""
        if self.connected:
            self.imu.disconnect()
            self.connected = False
            print("✓ Disconnected from IMU560")
    
    def magnetometer_calibration(self):
        """Interactive magnetometer calibration"""
        print("\n" + "="*50)
        print("MAGNETOMETER CALIBRATION")
        print("="*50)
        print("This calibration improves magnetic heading accuracy.")
        print("You will need to rotate the IMU in all directions.")
        print("\nIMPORTANT:")
        print("- Keep the IMU level during calibration")
        print("- Rotate slowly and smoothly")
        print("- Complete at least one full rotation")
        print("- Calibration should take at least 30 seconds")
        print("- Avoid magnetic interference sources")
        
        input("\nPress Enter to start calibration or Ctrl+C to cancel...")
        
        try:
            # Start calibration
            if not self.imu.start_magnetometer_calibration():
                print("✗ Failed to start magnetometer calibration")
                return False
            
            print("\n🔄 Calibration started!")
            print("Slowly rotate the IMU in all directions...")
            print("Status: ", end="", flush=True)
            
            # Calibration loop
            start_time = time.time()
            dots = 0
            
            while True:
                time.sleep(1)
                elapsed = time.time() - start_time
                
                # Show progress
                print(".", end="", flush=True)
                dots += 1
                if dots >= 50:  # New line every 50 seconds
                    print(f"\nElapsed: {elapsed:.0f}s - Status: ", end="", flush=True)
                    dots = 0
                
                # Check for user input (non-blocking)
                if elapsed >= 30:  # Minimum calibration time
                    print(f"\n\nMinimum calibration time reached ({elapsed:.0f}s)")
                    response = input("Enter 's' to stop calibration, 'a' to abandon, or Enter to continue: ").lower().strip()
                    
                    if response == 's':
                        break
                    elif response == 'a':
                        print("Abandoning calibration...")
                        if self.imu.abandon_magnetometer_calibration():
                            print("✓ Calibration abandoned")
                        else:
                            print("✗ Failed to abandon calibration")
                        return False
            
            # Stop and save calibration
            print("Stopping calibration...")
            if self.imu.stop_magnetometer_calibration():
                print("✓ Magnetometer calibration completed successfully!")
                
                # Save settings
                if self.imu.save_settings():
                    print("✓ Calibration settings saved to EEPROM")
                    return True
                else:
                    print("⚠ Calibration completed but failed to save to EEPROM")
                    return False
            else:
                print("✗ Failed to stop calibration")
                return False
                
        except KeyboardInterrupt:
            print("\n\nCalibration interrupted by user")
            print("Abandoning calibration...")
            if self.imu.abandon_magnetometer_calibration():
                print("✓ Calibration abandoned")
            else:
                print("✗ Failed to abandon calibration")
            return False
        except Exception as e:
            print(f"\n✗ Calibration error: {e}")
            return False
    
    def set_magnetic_declination(self):
        """Set magnetic declination for your location"""
        print("\n" + "="*50)
        print("MAGNETIC DECLINATION SETUP")
        print("="*50)
        print("Magnetic declination is the angle difference between")
        print("magnetic north and true north at your location.")
        print("\nYou can find your local magnetic declination at:")
        print("- https://www.magnetic-declination.com/")
        print("- https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml")
        
        try:
            # Get current declination
            current = self.imu.get_magnetic_declination()
            if current is not None:
                print(f"\nCurrent magnetic declination: {current:.2f}°")
            
            # Get new declination from user
            while True:
                try:
                    declination_str = input("\nEnter magnetic declination in degrees (or 'skip' to keep current): ").strip()
                    if declination_str.lower() == 'skip':
                        return True
                    
                    declination = float(declination_str)
                    if -180 <= declination <= 180:
                        break
                    else:
                        print("Please enter a value between -180 and 180 degrees")
                except ValueError:
                    print("Please enter a valid number")
            
            # Set declination
            if self.imu.set_magnetic_declination(declination):
                print(f"✓ Magnetic declination set to {declination:.2f}°")
                
                # Save settings
                if self.imu.save_settings():
                    print("✓ Settings saved to EEPROM")
                    return True
                else:
                    print("⚠ Declination set but failed to save to EEPROM")
                    return False
            else:
                print("✗ Failed to set magnetic declination")
                return False
                
        except KeyboardInterrupt:
            print("\nOperation cancelled by user")
            return False
        except Exception as e:
            print(f"✗ Error setting magnetic declination: {e}")
            return False
    
    def gravity_calibration(self):
        """Set gravity magnitude for your location"""
        print("\n" + "="*50)
        print("GRAVITY MAGNITUDE SETUP")
        print("="*50)
        print("Gravity varies slightly by location and altitude.")
        print("Standard gravity is 9.80665 m/s²")
        print("You can find local gravity at:")
        print("- https://www.ptb.de/cms/en/ptb/fachabteilungen/abt1/fb-11/ag-115/gravity-information-system.html")
        
        try:
            # Get current gravity
            current = self.imu.get_gravity_magnitude()
            if current is not None:
                print(f"\nCurrent gravity magnitude: {current:.5f} m/s²")
            
            # Get new gravity from user
            while True:
                try:
                    gravity_str = input("\nEnter gravity magnitude in m/s² (or 'skip' to keep current): ").strip()
                    if gravity_str.lower() == 'skip':
                        return True
                    
                    gravity = float(gravity_str)
                    if 9.0 <= gravity <= 10.0:  # Reasonable range for Earth
                        break
                    else:
                        print("Please enter a value between 9.0 and 10.0 m/s²")
                except ValueError:
                    print("Please enter a valid number")
            
            # Set gravity
            if self.imu.set_gravity_magnitude(gravity):
                print(f"✓ Gravity magnitude set to {gravity:.5f} m/s²")
                
                # Save settings
                if self.imu.save_settings():
                    print("✓ Settings saved to EEPROM")
                    return True
                else:
                    print("⚠ Gravity set but failed to save to EEPROM")
                    return False
            else:
                print("✗ Failed to set gravity magnitude")
                return False
                
        except KeyboardInterrupt:
            print("\nOperation cancelled by user")
            return False
        except Exception as e:
            print(f"✗ Error setting gravity magnitude: {e}")
            return False
    
    def run_full_calibration(self):
        """Run complete calibration sequence"""
        print("IMU560 Calibration Utility")
        print("=" * 30)
        
        if not self.connect():
            return False
        
        try:
            # Step 1: Gravity setup
            print("\n📍 Step 1: Gravity Setup")
            self.gravity_calibration()
            
            # Step 2: Magnetic declination
            print("\n🧭 Step 2: Magnetic Declination Setup")
            self.set_magnetic_declination()
            
            # Step 3: Magnetometer calibration
            print("\n🧲 Step 3: Magnetometer Calibration")
            self.magnetometer_calibration()
            
            print("\n" + "="*50)
            print("🎉 CALIBRATION COMPLETE!")
            print("="*50)
            print("Your IMU560 has been calibrated and is ready to use.")
            print("Settings have been saved to EEPROM.")
            
            return True
            
        finally:
            self.disconnect()

def main():
    """Main calibration function"""
    print("IMU560 Calibration Script")
    print("========================")
    
    if len(sys.argv) > 1:
        config_file = sys.argv[1]
        print(f"Using config file: {config_file}")
    else:
        config_file = "config.json"
        print(f"Using default config file: {config_file}")
    
    calibrator = IMUCalibrator(config_file)
    
    # Menu system
    while True:
        print("\n" + "="*40)
        print("CALIBRATION MENU")
        print("="*40)
        print("1. Full calibration (recommended)")
        print("2. Magnetometer calibration only")
        print("3. Set magnetic declination")
        print("4. Set gravity magnitude")
        print("5. Exit")
        
        try:
            choice = input("\nSelect option (1-5): ").strip()
            
            if choice == '1':
                calibrator.run_full_calibration()
            elif choice == '2':
                if calibrator.connect():
                    calibrator.magnetometer_calibration()
                    calibrator.disconnect()
            elif choice == '3':
                if calibrator.connect():
                    calibrator.set_magnetic_declination()
                    calibrator.disconnect()
            elif choice == '4':
                if calibrator.connect():
                    calibrator.gravity_calibration()
                    calibrator.disconnect()
            elif choice == '5':
                print("Exiting calibration utility")
                break
            else:
                print("Invalid choice. Please select 1-5.")
                
        except KeyboardInterrupt:
            print("\nExiting calibration utility")
            break
        except Exception as e:
            print(f"Error: {e}")

if __name__ == "__main__":
    main()