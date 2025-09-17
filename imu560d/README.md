# IMU560 Combined Inertial Navigation System Controller

A comprehensive Python library for controlling the IMU560 Combined Inertial Navigation System with support for accelerometer, gyroscope, magnetometer, GPS, and barometric sensors.

## Features

- ✅ Full IMU560 protocol implementation
- ✅ Support for all sensor data types (accelerometer, gyroscope, magnetometer, GPS, barometric)
- ✅ Continuous and query-response modes
- ✅ Magnetometer calibration utilities
- ✅ Configurable output masks
- ✅ Context manager support
- ✅ Comprehensive logging
- ✅ Thread-safe continuous data reading
- ✅ CRC validation for reliable communication

## Requirements

- Python 3.7+
- pyserial

## Installation

1. Clone or download this repository
2. Install dependencies:
```bash
pip install pyserial



## API Reference

### Main Class: IMU560

#### Connection and Setup Methods

- **`__init__(config_file="config.json")`** - Initialize IMU560 controller with configuration file
- **`connect(port=None, baudrate=None)`** - Connect to IMU560 via serial port
- **`disconnect()`** - Disconnect from IMU560 and cleanup resources
- **`load_config(config_file)`** - Load configuration from JSON file
- **`setup_logging()`** - Setup logging configuration from config file

#### Core Communication Methods

- **`calculate_crc(data)`** - Calculate CRC16 checksum for data validation
- **`build_frame(cmd, data=b'')`** - Build complete command frame with CRC
- **`parse_frame(data)`** - Parse received frame and validate CRC
- **`send_command(cmd, data=b'')`** - Send command and receive response
- **`read_frame()`** - Read complete frame from serial port

#### Data Reading Methods

- **`get_sensor_data(mask=None)`** - Get sensor data with specified or default mask (returns SensorData object)
- **`parse_sensor_data(data, mask)`** - Parse raw sensor data based on output mask
- **`start_continuous_mode(callback=None, frequency_divider=1)`** - Start continuous data output mode
- **`stop_continuous_mode()`** - Stop continuous data output mode

#### Output Configuration Methods

- **`set_output_mask(mask)`** - Set default output data mask (which sensors to read)
- **`get_output_mask()`** - Get current default output mask
- **`set_continuous_mode(enabled, frequency_divider)`** - Configure continuous output mode and frequency
- **`get_continuous_mode()`** - Get current continuous mode settings

#### Protocol and Communication Settings

- **`set_baudrate(baudrate)`** - Set serial communication baudrate (9600-230400)
- **`get_baudrate()`** - Get current communication baudrate

#### Navigation and GPS Settings

- **`set_gravity_magnitude(gravity=9.8)`** - Set local gravity magnitude in m/s²
- **`get_gravity_magnitude()`** - Get current gravity magnitude setting
- **`set_magnetic_declination(declination)`** - Set magnetic declination angle in degrees
- **`get_magnetic_declination()`** - Get current magnetic declination setting

#### Magnetometer Calibration Methods

- **`start_magnetometer_calibration()`** - Start magnetometer horizontal calibration process
- **`stop_magnetometer_calibration()`** - Stop and save magnetometer calibration
- **`abandon_magnetometer_calibration()`** - Abandon calibration without saving

#### System Control Methods

- **`save_settings()`** - Save current settings to EEPROM (non-volatile memory)
- **`soft_reset()`** - Perform software reset of the IMU

#### Context Manager Methods

- **`__enter__()`** - Context manager entry (enables `with` statement)
- **`__exit__(exc_type, exc_val, exc_tb)`** - Context manager exit with cleanup

### Data Classes and Enums

#### SensorData Class
Data container with all sensor readings:

**Attitude (radians):**
- `roll`, `pitch`, `yaw` - Euler angles

**Motion Sensors:**
- `gyro_x`, `gyro_y`, `gyro_z` - Angular rates (rad/s)
- `accel_x`, `accel_y`, `accel_z` - Linear acceleration (m/s²)
- `mag_x`, `mag_y`, `mag_z` - Magnetic field (µT)

**Environmental:**
- `temp_0`, `temp_1` - Temperature sensors (°C)
- `baro_altitude` - Barometric altitude (cm)
- `baro_pressure` - Barometric pressure (Pa)

**Navigation:**
- `latitude`, `longitude`, `altitude` - GPS position (degrees, degrees, meters)
- `vel_north`, `vel_east`, `vel_down` - Velocity components (m/s)
- `mag_heading` - Magnetic heading (degrees)

**Time and Status:**
- `time_since_reset` - Time since reset (ms)
- `gps_itow`, `gps_flags`, `gps_num_sv` - GPS status info
- `utc_year`, `utc_month`, `utc_day`, `utc_hour`, `utc_minute`, `utc_second` - UTC time

#### OutputMask Enum
Bit flags for selecting output data:

**Attitude:**
- `OutputMask.EULER` - Roll, pitch, yaw angles
- `OutputMask.MAG_HEADING` - Magnetic compass heading

**Motion Sensors:**
- `OutputMask.GYROSCOPES` - 3-axis gyroscope data
- `OutputMask.ACCELEROMETERS` - 3-axis accelerometer data
- `OutputMask.MAGNETOMETERS` - 3-axis magnetometer data

**Environmental:**
- `OutputMask.TEMPERATURES` - Temperature sensor data
- `OutputMask.BARO_ALTITUDE` - Barometric altitude
- `OutputMask.BARO_PRESSURE` - Barometric pressure

**Navigation:**
- `OutputMask.GPS_INFO` - GPS status information
- `OutputMask.POSITION` - GPS position (lat/lon/alt)
- `OutputMask.VELOCITY` - GPS velocity components
- `OutputMask.UTC_TIME_REFERENCE` - UTC time from GPS

**System:**
- `OutputMask.TIME_SINCE_RESET` - System uptime

#### Commands Enum
Internal command codes for IMU communication (for advanced users)

#### ErrorCode Enum
IMU error response codes:
- `ErrorCode.NO_ERROR` (0x00) - Command executed successfully
- `ErrorCode.ERROR` (0x01) - General command error
- `ErrorCode.INVALID_FRAME` (0x04) - Invalid command frame
- `ErrorCode.INVALID_PARAMETER` (0x09) - Invalid parameter value
- `ErrorCode.NOT_READY` (0x0A) - Sensor not ready

### Utility Functions and Constants

- **`DEFAULT_OUTPUT_MASK`** - Default output mask with commonly used sensors
- **`IMUError`** - Custom exception class for IMU-related errors

### Usage Patterns

#### Basic Usage Pattern
```python
with IMU560() as imu:
    if imu.connect():
        data = imu.get_sensor_data()
        # Process data
Configuration Pattern
python
with IMU560() as imu:
    if imu.connect():
        imu.set_output_mask(custom_mask)
        imu.set_continuous_mode(True, 1)  # 100Hz
        imu.save_settings()
Calibration Pattern
python
with IMU560() as imu:
    if imu.connect():
        imu.start_magnetometer_calibration()
        # Rotate device for 30+ seconds
        imu.stop_magnetometer_calibration()
        imu.save_settings()
Continuous Data Pattern
python
def callback(data):
    print(f"Roll: {data.roll:.3f}")

with IMU560() as imu:
    if imu.connect():
        imu.start_continuous_mode(callback, frequency_divider=1)
        time.sleep(10)  # Run for 10 seconds
text

This comprehensive API reference covers all the functions in the IMU560 class with their purposes, parameters, and usage patterns.
