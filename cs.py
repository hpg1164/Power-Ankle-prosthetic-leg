import serial
import csv

# Configure the serial port (Update COM port and baud rate as per your setup)
ser = serial.Serial('COM4', 115200, timeout=1)  # Change COM3 to your port

# File to store data
csv_filename = "imu_data.csv"

# Open CSV file and write the header
with open(csv_filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Time", "Axis_X", "Axis_Y", "Axis_Z"])  # Change headers if needed

    try:
        while True:
            line = ser.readline().decode('utf-8').strip()  # Read and decode serial data
            if line:
                values = line.split(',')  # Assuming values are comma-separated
                if len(values) == 4:  # Ensure correct number of columns
                    writer.writerow(values)
                    print(values)  # Optional: Print in terminal

    except KeyboardInterrupt:
        print("\nStopped by user.")
        ser.close()
