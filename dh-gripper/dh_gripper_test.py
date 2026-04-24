import serial
import struct
import time

PORT = '/dev/ttyUSB0'
BAUDRATE = 115200
DEVICE_ID = 0x01


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def write_reg(ser, reg, value):
    frame = struct.pack('>BBHH', DEVICE_ID, 0x06, reg, value)
    frame += struct.pack('<H', crc16(frame))
    ser.write(frame)
    time.sleep(0.05)
    ser.read(8)  # echo


if __name__ == '__main__':
    with serial.Serial(PORT, BAUDRATE, timeout=0.5) as ser:
        print("Initializing...")
        write_reg(ser, 0x0100, 0x01)
        time.sleep(3.0)

        print("Opening...")
        write_reg(ser, 0x0101, 50)    # force 50%
        write_reg(ser, 0x0103, 0)     # position 0 = open
        time.sleep(3.0)

        print("Closing...")
        write_reg(ser, 0x0103, 1000)  # position 1000 = closed
        time.sleep(3.0)

        print("Done.")
