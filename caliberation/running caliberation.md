3. Usage Instructions
Installation
bash


pip install pyserial


Running the System
Test IMU Connection:
bash

python imu_calibration_system.py --test-imu


Test Pan-Tilt Connection:
bash

python imu_calibration_system.py --test-pantilt


Perform Full Calibration:
bash

python imu_calibration_system.py --calibrate


Custom Configuration:
bash

python imu_calibration_system.py --config my_config.json --calibrate

Configuration Options

Edit the config.json file to customize:

Serial ports and baud rates for both devices

Calibration parameters (rotation time, steps, tilt positions)

Pan-tilt speeds and movement settings

Logging levels and output files

Features

✅ Automatic IMU Protocol Handling: Implements the complete IMU560 communication protocol with CRC validation

✅ Intelligent Pan-Tilt Control: Synchronized movement control for optimal calibration coverage

✅ Multi-Position Calibration: Calibrates at different tilt angles for comprehensive magnetic field mapping

✅ Real-Time Monitoring: Live display of orientation data during calibration

✅ Error Handling: Robust error detection and recovery mechanisms

✅ Configurable Parameters: JSON-based configuration for easy customization

✅ Safety Features: Interrupt handling and automatic cleanup

✅ Comprehensive Testing: Built-in test modes for device validation

The system will automatically rotate the platform through 360 degrees at multiple tilt angles, collecting magnetic field data for accurate compass calibration. The entire process takes approximately 2-3 minutes depending on your configuration settings.