import serial

def calc_crc(data):
    """Calculate 16-bit CRC (sum) for IMU560 frame."""
    crc = 0
    for b in data:
        crc = (crc + b) & 0xFFFF
    return crc

def build_frame(data_byte):
    """Build IMU560 calibration frame with CRC and ending byte."""
    frame = bytearray([0xFF, 0x02, 0x70, 0x00, 0x01, data_byte])
    crc = calc_crc(frame[2:])  # calculate CRC from CMD to last data byte
    frame += bytearray([crc & 0xFF, (crc >> 8) & 0xFF, 0x03])
    return frame

def send_command(ser, mode):
    """Send calibration command and check response."""
    if mode == "start":
        frame = build_frame(0x08)
        msg = "START calibration"
    elif mode == "stop":
        frame = build_frame(0x0A)
        msg = "STOP calibration"
    elif mode == "abandon":
        frame = build_frame(0x09)
        msg = "ABANDON calibration"
    else:
        print("❌ Unknown mode")
        return

    # Send packet
    print(f"➡️ Sending {msg} command...")
    print("📤 TX Packet:", frame.hex(" "))
    ser.write(frame)

    # Read response
    resp = ser.read(20)
    if resp:
        print("📥 RX Packet:", resp.hex(" "))

        # IMU ACK format: FF 02 01 00 01 00 05 63 03 (success)
        if resp[2] == 0x01 and resp[5] == 0x00:
            if mode == "start":
                print("✅ Successfully entered calibration mode. Rotate IMU one full revolution (~30s).")
            elif mode == "stop":
                print("✅ Calibration stopped and parameters saved.")
            elif mode == "abandon":
                print("⚠️ Calibration abandoned, no data saved.")
        else:
            print("⚠️ IMU returned an error response.")
    else:
        print("⚠️ No response received from IMU. Check wiring/port.")

if __name__ == "__main__":
    # Open serial port on Raspberry Pi (adjust if needed)
    ser = serial.Serial("/dev/ttyUSB1", 115200, timeout=1)

    # Example: Start calibration
    send_command(ser, "start")

    # Later, run stop or abandon depending on what you want
    # send_command(ser, "stop")
    # send_command(ser, "abandon")

    ser.close()
