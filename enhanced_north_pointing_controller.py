#!/usr/bin/env python3
"""
Enhanced SP2520 North Pointing Controller
Advanced PTZ control with improved timing and bidirectional movement for accurate north pointing
"""
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), 'sp250'))
sys.path.append(os.path.join(os.path.dirname(__file__), 'imu560d'))

import time
import math
import threading
from gps_magnetometer_reader import GPSMagnetometerReader
from sp2520_controller import SP2520Controller

class EnhancedNorthPointingController:
    """Enhanced controller for precise north pointing with improved timing"""
    
    def __init__(self, sp2520_port='/dev/ttyUSB1', imu_config="imu560d/config.json"):
        """Initialize the enhanced north pointing controller"""
        
        # Initialize devices
        self.sp2520 = SP2520Controller(port=sp2520_port)
        self.gps_mag_reader = GPSMagnetometerReader(imu_config)
        
        # Enhanced control parameters
        self.target_heading = 0.0  # North
        self.heading_tolerance = 2.0  # degrees (±2°)
        self.fine_tolerance = 0.5  # degrees for fine adjustment
        self.max_correction_angle = 10.0  # Maximum correction per step
        self.update_interval = 0.5  # seconds between corrections (faster)
        
        # Movement timing parameters (calibrated for better accuracy)
        self.degrees_per_second = 8.0  # Estimated degrees per second at default speed
        self.min_movement_time = 0.05  # Minimum movement time
        self.max_movement_time = 3.0   # Maximum movement time
        self.stabilization_time = 0.3  # Time to wait after movement
        
        # Bidirectional movement parameters
        self.use_bidirectional = True  # Use both directions for better timing
        self.overshoot_compensation = 0.5  # degrees to compensate for overshoot
        
        # State variables
        self.running = False
        self.current_heading = None
        self.control_thread = None
        self.movement_history = []  # Track movement history for learning
        self.calibration_data = {}
        
        print("🎯 Enhanced North Pointing Controller Initialized")
    
    def connect_devices(self):
        """Connect to both SP2520 and GPS/Magnetometer reader"""
        print("🔄 Connecting to devices...")
        
        # Connect to GPS/Magnetometer reader
        if not self.gps_mag_reader.connect():
            print("❌ Failed to connect to IMU560")
            return False
        print("✅ Connected to IMU560")
        
        # Test SP2520 connection
        try:
            self.sp2520.stop()  # Test command
            print("✅ Connected to SP2520")
        except Exception as e:
            print(f"❌ Failed to connect to SP2520: {e}")
            return False
        
        print("✅ All devices connected and configured")
        return True
    
    def get_current_heading(self):
        """Get current magnetic heading with error handling"""
        try:
            mag_data = self.gps_mag_reader.get_magnetometer_readings()
            if mag_data and mag_data['true_heading'] is not None:
                return mag_data['true_heading']
            return None
        except Exception as e:
            print(f"❌ Error reading heading: {e}")
            return None
    
    def calculate_correction(self, current_heading):
        """
        Calculate precise correction needed to point north
        
        Args:
            current_heading: Current heading in degrees (0-360)
            
        Returns:
            tuple: (correction_angle, direction, movement_type)
        """
        if current_heading is None:
            return 0, 'none', 'none'
        
        # Calculate shortest rotation to north (0°)
        error = current_heading - self.target_heading
        
        # Normalize error to [-180, 180]
        while error > 180:
            error -= 360
        while error < -180:
            error += 360
        
        # Determine movement type based on error magnitude
        abs_error = abs(error)
        
        if abs_error <= self.fine_tolerance:
            return 0, 'none', 'none'
        elif abs_error <= self.heading_tolerance:
            movement_type = 'fine'
        else:
            movement_type = 'coarse'
        
        # Limit correction angle
        correction_angle = min(abs_error, self.max_correction_angle)
        
        # Apply overshoot compensation for fine movements
        if movement_type == 'fine' and self.use_bidirectional:
            correction_angle = max(correction_angle - self.overshoot_compensation, 0.1)
        
        # Determine direction
        if error > 0:
            direction = 'left'  # Rotate counter-clockwise
        else:
            direction = 'right'  # Rotate clockwise
        
        return correction_angle, direction, movement_type
    
    def calculate_movement_time(self, correction_angle, movement_type):
        """
        Calculate precise movement time based on correction angle and type
        
        Args:
            correction_angle: Degrees to rotate
            movement_type: 'coarse' or 'fine'
            
        Returns:
            float: Movement time in seconds
        """
        # Base calculation
        base_time = correction_angle / self.degrees_per_second
        
        # Adjust for movement type
        if movement_type == 'fine':
            # Slower, more precise movement for fine adjustments
            base_time *= 1.5
            # Use slower speed for fine movements
            self.sp2520.set_speed(pan_speed=0x15, tilt_speed=0x15)
        else:
            # Normal speed for coarse movements
            self.sp2520.set_speed(pan_speed=0x25, tilt_speed=0x25)
        
        # Apply limits
        movement_time = max(self.min_movement_time, min(base_time, self.max_movement_time))
        
        return movement_time
    
    def apply_correction(self, correction_angle, direction, movement_type):
        """
        Apply correction to SP2520 with enhanced timing and bidirectional support
        
        Args:
            correction_angle: Degrees to rotate
            direction: 'left' or 'right'
            movement_type: 'coarse' or 'fine'
            
        Returns:
            bool: Success status
        """
        if direction == 'none' or correction_angle == 0:
            return True
        
        try:
            # Calculate movement time
            movement_time = self.calculate_movement_time(correction_angle, movement_type)
            
            print(f"🔄 {movement_type.title()} correction: {correction_angle:.1f}° {direction} ({movement_time:.2f}s)")
            
            # Record movement start
            start_heading = self.get_current_heading()
            start_time = time.time()
            
            # Apply primary movement
            if direction == 'left':
                self.sp2520.move_left()
            else:
                self.sp2520.move_right()
            
            # Move for calculated time
            time.sleep(movement_time)
            
            # Stop movement
            self.sp2520.stop()
            
            # Wait for stabilization
            time.sleep(self.stabilization_time)
            
            # Check result and apply bidirectional correction if needed
            if self.use_bidirectional and movement_type == 'coarse':
                end_heading = self.get_current_heading()
                if end_heading is not None and start_heading is not None:
                    actual_movement = self._calculate_actual_movement(start_heading, end_heading, direction)
                    overshoot = actual_movement - correction_angle
                    
                    # Apply counter-correction if significant overshoot
                    if abs(overshoot) > 1.0:
                        counter_direction = 'right' if direction == 'left' else 'left'
                        counter_time = abs(overshoot) / self.degrees_per_second * 0.8  # Slightly less to avoid oscillation
                        counter_time = max(self.min_movement_time, min(counter_time, 1.0))
                        
                        print(f"🔧 Counter-correction: {abs(overshoot):.1f}° {counter_direction} ({counter_time:.2f}s)")
                        
                        if counter_direction == 'left':
                            self.sp2520.move_left()
                        else:
                            self.sp2520.move_right()
                        
                        time.sleep(counter_time)
                        self.sp2520.stop()
                        time.sleep(self.stabilization_time)
            
            # Record movement for learning
            end_time = time.time()
            self._record_movement(start_heading, correction_angle, direction, movement_type, 
                                movement_time, start_time, end_time)
            
            return True
            
        except Exception as e:
            print(f"❌ Error applying correction: {e}")
            self.sp2520.stop()  # Ensure it stops on error
            return False
    
    def _calculate_actual_movement(self, start_heading, end_heading, direction):
        """Calculate actual movement that occurred"""
        if start_heading is None or end_heading is None:
            return 0
        
        movement = end_heading - start_heading
        
        # Normalize to [0, 360]
        while movement < 0:
            movement += 360
        while movement >= 360:
            movement -= 360
        
        # Convert to signed movement based on direction
        if direction == 'left':
            if movement > 180:
                movement = movement - 360
        else:  # right
            if movement > 180:
                movement = 360 - movement
            else:
                movement = -movement
        
        return abs(movement)
    
    def _record_movement(self, start_heading, target_angle, direction, movement_type, 
                        movement_time, start_time, end_time):
        """Record movement data for learning and calibration"""
        movement_record = {
            'timestamp': start_time,
            'start_heading': start_heading,
            'target_angle': target_angle,
            'direction': direction,
            'movement_type': movement_type,
            'movement_time': movement_time,
            'duration': end_time - start_time
        }
        
        self.movement_history.append(movement_record)
        
        # Keep only recent history (last 50 movements)
        if len(self.movement_history) > 50:
            self.movement_history.pop(0)
    
    def enhanced_control_loop(self):
        """Enhanced control loop with adaptive timing"""
        print("🎯 Starting enhanced north pointing control loop")
        
        consecutive_errors = 0
        max_errors = 5
        successful_corrections = 0
        
        while self.running:
            try:
                # Get current heading
                current_heading = self.get_current_heading()
                self.current_heading = current_heading
                
                if current_heading is not None:
                    consecutive_errors = 0  # Reset error count
                    
                    # Calculate required correction
                    correction_angle, direction, movement_type = self.calculate_correction(current_heading)
                    
                    # Apply correction if needed
                    if direction != 'none':
                        if self.apply_correction(correction_angle, direction, movement_type):
                            successful_corrections += 1
                            print(f"✅ Correction #{successful_corrections} applied: {correction_angle:.1f}° {direction} ({movement_type})")
                        else:
                            print("❌ Failed to apply correction")
                    else:
                        deviation = abs(current_heading) if current_heading <= 180 else abs(current_heading - 360)
                        print(f"🎯 On target: {current_heading:.1f}° (deviation: {deviation:.1f}°)")
                
                else:
                    consecutive_errors += 1
                    print(f"❌ Could not read heading (error {consecutive_errors}/{max_errors})")
                    
                    if consecutive_errors >= max_errors:
                        print("❌ Too many consecutive errors, stopping control loop")
                        break
                
                # Adaptive update interval based on accuracy
                if current_heading is not None:
                    deviation = abs(current_heading) if current_heading <= 180 else abs(current_heading - 360)
                    if deviation <= self.fine_tolerance:
                        time.sleep(self.update_interval * 2)  # Slower updates when on target
                    elif deviation <= self.heading_tolerance:
                        time.sleep(self.update_interval * 1.5)  # Medium updates for fine adjustments
                    else:
                        time.sleep(self.update_interval)  # Fast updates for coarse corrections
                else:
                    time.sleep(1)  # Wait longer on errors
                
            except Exception as e:
                print(f"❌ Control loop error: {e}")
                consecutive_errors += 1
                if consecutive_errors >= max_errors:
                    break
                time.sleep(1)
        
        print(f"🛑 Enhanced control loop stopped (Total corrections: {successful_corrections})")
    
    def start_north_pointing(self):
        """Start enhanced automatic north pointing"""
        if self.running:
            print("⚠️ North pointing already active")
            return False
        
        print("🚀 Starting enhanced automatic north pointing...")
        self.running = True
        
        # Start enhanced control loop in background thread
        self.control_thread = threading.Thread(target=self.enhanced_control_loop)
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
    
    def manual_precise_point_north(self):
        """Manually point to north with enhanced precision"""
        print("🎯 Manual precise north pointing...")
        
        # Get current heading
        current_heading = self.get_current_heading()
        if current_heading is None:
            print("❌ Could not read current heading")
            return False
        
        print(f"📍 Current heading: {current_heading:.1f}°")
        
        # Multi-step correction for better accuracy
        max_iterations = 5
        iteration = 0
        
        while iteration < max_iterations:
            iteration += 1
            current_heading = self.get_current_heading()
            
            if current_heading is None:
                print("❌ Lost heading signal")
                return False
            
            correction_angle, direction, movement_type = self.calculate_correction(current_heading)
            
            if direction == 'none':
                deviation = abs(current_heading) if current_heading <= 180 else abs(current_heading - 360)
                print(f"🎯 Successfully pointed north! (Final deviation: {deviation:.1f}°)")
                return True
            
            print(f"🔄 Iteration {iteration}: {correction_angle:.1f}° {direction} correction needed")
            
            if not self.apply_correction(correction_angle, direction, movement_type):
                print(f"❌ Failed at iteration {iteration}")
                return False
            
            # Brief pause between iterations
            time.sleep(0.5)
        
        # Final check
        final_heading = self.get_current_heading()
        if final_heading is not None:
            deviation = abs(final_heading) if final_heading <= 180 else abs(final_heading - 360)
            print(f"🎯 Completed after {iteration} iterations (Final deviation: {deviation:.1f}°)")
            return deviation <= self.heading_tolerance
        
        return False
    
    def calibrate_movement_timing(self):
        """Calibrate movement timing for better accuracy"""
        print("🔧 Starting movement timing calibration...")
        
        calibration_angles = [5, 10, 15, 20]  # Test angles
        directions = ['left', 'right']
        
        for angle in calibration_angles:
            for direction in directions:
                print(f"\n🔄 Calibrating {angle}° {direction} movement...")
                
                # Get initial heading
                start_heading = self.get_current_heading()
                if start_heading is None:
                    continue
                
                # Apply movement
                movement_time = self.calculate_movement_time(angle, 'coarse')
                
                if direction == 'left':
                    self.sp2520.move_left()
                else:
                    self.sp2520.move_right()
                
                time.sleep(movement_time)
                self.sp2520.stop()
                time.sleep(self.stabilization_time)
                
                # Measure actual movement
                end_heading = self.get_current_heading()
                if end_heading is not None:
                    actual_movement = self._calculate_actual_movement(start_heading, end_heading, direction)
                    accuracy = (actual_movement / angle) * 100
                    
                    print(f"📊 Target: {angle}°, Actual: {actual_movement:.1f}°, Accuracy: {accuracy:.1f}%")
                    
                    # Store calibration data
                    key = f"{angle}_{direction}"
                    self.calibration_data[key] = {
                        'target': angle,
                        'actual': actual_movement,
                        'accuracy': accuracy,
                        'movement_time': movement_time
                    }
                
                time.sleep(2)  # Pause between tests
        
        # Calculate average accuracy
        if self.calibration_data:
            avg_accuracy = sum(data['accuracy'] for data in self.calibration_data.values()) / len(self.calibration_data)
            print(f"\n📈 Average calibration accuracy: {avg_accuracy:.1f}%")
            
            # Adjust degrees_per_second based on calibration
            if avg_accuracy < 90:
                adjustment_factor = avg_accuracy / 100
                self.degrees_per_second *= adjustment_factor
                print(f"🔧 Adjusted movement speed to {self.degrees_per_second:.1f} degrees/second")
        
        print("✅ Calibration completed")
    
    def get_status(self):
        """Get enhanced status information"""
        status = {
            'running': self.running,
            'current_heading': self.current_heading,
            'target_heading': self.target_heading,
            'heading_tolerance': self.heading_tolerance,
            'fine_tolerance': self.fine_tolerance,
            'degrees_per_second': self.degrees_per_second,
            'use_bidirectional': self.use_bidirectional,
            'movement_history_count': len(self.movement_history),
            'calibration_data_count': len(self.calibration_data)
        }
        
        # Add GPS data if available
        try:
            gps_data = self.gps_mag_reader.get_gps_coordinates()
            if gps_data:
                status['gps'] = {
                    'latitude': gps_data['latitude'],
                    'longitude': gps_data['longitude'],
                    'altitude': gps_data['altitude'],
                    'satellites': gps_data['gps_info']['num_satellites']
                }
        except:
            pass
        
        return status
    
    def interactive_mode(self):
        """Enhanced interactive control interface"""
        while True:
            print("\n" + "="*70)
            print("🎯 ENHANCED SP2520 NORTH POINTING CONTROLLER")
            print("="*70)
            print("1. Manual Precise Point North")
            print("2. Start Enhanced Automatic North Pointing")
            print("3. Stop Automatic North Pointing")
            print("4. Show Enhanced Status")
            print("5. Calibrate Movement Timing")
            print("6. Set Control Parameters")
            print("7. Test Current Heading & GPS")
            print("8. View Movement History")
            print("9. Exit")
            
            try:
                choice = input("\nSelect option (1-9): ").strip()
                
                if choice == '1':
                    self.manual_precise_point_north()
                
                elif choice == '2':
                    if self.start_north_pointing():
                        print("✅ Enhanced automatic north pointing started")
                        print("Press option 3 to stop")
                
                elif choice == '3':
                    self.stop_north_pointing()
                
                elif choice == '4':
                    status = self.get_status()
                    print(f"\n📊 ENHANCED STATUS:")
                    print(f"Running: {'✅ Yes' if status['running'] else '❌ No'}")
                    if status['current_heading'] is not None:
                        deviation = abs(status['current_heading']) if status['current_heading'] <= 180 else abs(status['current_heading'] - 360)
                        print(f"Current Heading: {status['current_heading']:.1f}°")
                        print(f"Deviation from North: {deviation:.1f}°")
                    print(f"Tolerance: ±{status['heading_tolerance']:.1f}° (Fine: ±{status['fine_tolerance']:.1f}°)")
                    print(f"Movement Speed: {status['degrees_per_second']:.1f}°/s")
                    print(f"Bidirectional: {'✅ Enabled' if status['use_bidirectional'] else '❌ Disabled'}")
                    print(f"Movement History: {status['movement_history_count']} records")
                    
                    if 'gps' in status:
                        gps = status['gps']
                        print(f"📍 GPS: {gps['latitude']:.6f}, {gps['longitude']:.6f}")
                        print(f"🛰️ Altitude: {gps['altitude']:.1f}m, Satellites: {gps['satellites']}")
                
                elif choice == '5':
                    self.calibrate_movement_timing()
                
                elif choice == '6':
                    print(f"\nCurrent Parameters:")
                    print(f"Heading Tolerance: ±{self.heading_tolerance:.1f}°")
                    print(f"Fine Tolerance: ±{self.fine_tolerance:.1f}°")
                    print(f"Max Correction: {self.max_correction_angle:.1f}°")
                    print(f"Update Interval: {self.update_interval:.1f}s")
                    print(f"Movement Speed: {self.degrees_per_second:.1f}°/s")
                    print(f"Bidirectional: {'Enabled' if self.use_bidirectional else 'Disabled'}")
                    
                    try:
                        tol = input("New heading tolerance (degrees, Enter to skip): ").strip()
                        fine_tol = input("New fine tolerance (degrees, Enter to skip): ").strip()
                        max_corr = input("New max correction (degrees, Enter to skip): ").strip()
                        interval = input("New update interval (seconds, Enter to skip): ").strip()
                        speed = input("New movement speed (degrees/second, Enter to skip): ").strip()
                        bidir = input("Enable bidirectional (y/n, Enter to skip): ").strip().lower()
                        
                        if tol:
                            self.heading_tolerance = max(0.1, min(float(tol), 10.0))
                        if fine_tol:
                            self.fine_tolerance = max(0.1, min(float(fine_tol), 5.0))
                        if max_corr:
                            self.max_correction_angle = max(1.0, min(float(max_corr), 30.0))
                        if interval:
                            self.update_interval = max(0.1, min(float(interval), 5.0))
                        if speed:
                            self.degrees_per_second = max(1.0, min(float(speed), 20.0))
                        if bidir in ['y', 'yes']:
                            self.use_bidirectional = True
                        elif bidir in ['n', 'no']:
                            self.use_bidirectional = False
                        
                        print("✅ Parameters updated")
                    except ValueError:
                        print("❌ Invalid input")
                
                elif choice == '7':
                    # Test heading and GPS
                    mag_data = self.gps_mag_reader.get_magnetometer_readings()
                    gps_data = self.gps_mag_reader.get_gps_coordinates()
                    
                    if mag_data:
                        print(f"\n🧭 Magnetometer:")
                        print(f"True Heading: {mag_data['true_heading']:.1f}°")
                        print(f"Magnetic Heading: {mag_data['magnetic_heading']:.1f}°")
                        print(f"Field Strength: {mag_data['field_strength']:.1f} µT")
                        deviation = abs(mag_data['true_heading']) if mag_data['true_heading'] <= 180 else abs(mag_data['true_heading'] - 360)
                        print(f"Deviation from North: {deviation:.1f}°")
                    
                    if gps_data:
                        print(f"\n📍 GPS:")
                        print(f"Position: {gps_data['latitude']:.6f}, {gps_data['longitude']:.6f}")
                        print(f"Altitude: {gps_data['altitude']:.1f}m")
                        print(f"Satellites: {gps_data['gps_info']['num_satellites']}")
                
                elif choice == '8':
                    if self.movement_history:
                        print(f"\n📈 Recent Movement History (Last {min(10, len(self.movement_history))}):")
                        for i, record in enumerate(self.movement_history[-10:], 1):
                            print(f"{i:2d}. {record['target_angle']:4.1f}° {record['direction']:5s} "
                                  f"({record['movement_type']:6s}) - {record['movement_time']:.2f}s")
                    else:
                        print("📈 No movement history available")
                
                elif choice == '9':
                    break
                
                else:
                    print("Invalid choice. Please select 1-9.")
                
                if choice != '9':
                    input("\nPress Enter to continue...")
                
            except KeyboardInterrupt:
                print("\nExiting...")
                break
    
    def disconnect(self):
        """Clean up and disconnect"""
        self.stop_north_pointing()
        self.sp2520.close()
        self.gps_mag_reader.disconnect()
        print("✅ Disconnected from all devices")

def main():
    """Main function"""
    print("Enhanced SP2520 North Pointing Controller")
    print("========================================")
    
    # Adjust these ports as needed for your system
    controller = EnhancedNorthPointingController(
        sp2520_port='/dev/ttyUSB1',  # Adjust as needed
        imu_config="imu560d/config.json"
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