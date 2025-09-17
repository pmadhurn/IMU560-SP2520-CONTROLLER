import os
import subprocess

def get_usb_devices():
    devices = []
    base_paths = ['/dev/ttyUSB', '/dev/ttyACM']

    for base in base_paths:
        for i in range(10):
            dev_path = f"{base}{i}"
            if os.path.exists(dev_path):
                try:
                    # Use udevadm to get device properties
                    output = subprocess.check_output(['udevadm', 'info', '--query=all', '--name', dev_path], encoding='utf-8')
                    
                    # Collect relevant properties
                    device_info = {
                        'dev': dev_path,
                        'ID_VENDOR_ID': '',
                        'ID_MODEL_ID': '',
                        'ID_MODEL': '',
                        'ID_SERIAL_SHORT': ''
                    }

                    for line in output.split('\n'):
                        for key in device_info:
                            if key + '=' in line:
                                device_info[key] = line.split('=')[1]
                    
                    devices.append(device_info)

                except subprocess.CalledProcessError:
                    continue
    return devices

def identify_devices(devices):
    mpu_port = None
    tilt_port = None

    for dev in devices:
        vendor = dev['ID_VENDOR_ID']
        model = dev['ID_MODEL_ID']
        name = dev['ID_MODEL']
        serial = dev['ID_SERIAL_SHORT']

        # Customize this logic based on your real devices
        if 'FT232' in name or 'FTDI' in name:
            if not mpu_port:
                mpu_port = dev['dev']
            else:
                tilt_port = dev['dev']
        elif 'MPU' in name:
            mpu_port = dev['dev']
        elif 'Tilt' in name:
            tilt_port = dev['dev']

    return mpu_port, tilt_port

if __name__ == "__main__":
    devices = get_usb_devices()
    mpu_port, tilt_port = identify_devices(devices)

    if mpu_port:
        print(f"MPU device is connected at: {mpu_port}")
    else:
        print("MPU device not found.")

    if tilt_port:
        print(f"Tilt machine is connected at: {tilt_port}")
    else:
        print("Tilt machine device not found.")
