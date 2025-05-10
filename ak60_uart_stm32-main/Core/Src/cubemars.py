import serial
import time

# Configure the serial port
serial_port = '/dev/ttyUSB0'  # Change this to your serial port
baud_rate = 921600              # Set the baud rate according to your device

# Create a serial connection
with serial.Serial(serial_port, baud_rate, timeout=1) as ser:
    # Wait for the serial connection to initialize
    time.sleep(2)

    # Prepare the hex values to send
    # hex_values = [0x02, 0x05 , 0x09 , 0x05 , 0x5D , 0x4A , 0x80 , 0x7B , 0x29 , 0x03]
    # hex_values = [0x02, 0x05, 0x09, 0x0A, 0xBA, 0x95, 0x00, 0x1E, 0xE7, 0x03]
    hex_values = [0x02 , 0x05 , 0x08 , 0x00 , 0x00 , 0x03 , 0xE8 , 0x2B , 0x58 , 0x03 ]

    byte_data = bytearray(hex_values)

    # Send the data
    ser.write(byte_data)
    print(f"Sent: {byte_data.hex().upper()}")

    # Read the response from the serial port
    time.sleep(1)  # Wait for a moment to receive data
    if ser.in_waiting > 0:
        response = ser.read(ser.in_waiting)
        print(f"Received: {response.hex().upper()}")
    else:
        print("No data received.")
