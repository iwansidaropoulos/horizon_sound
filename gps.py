import serial
import re

# Set the serial port and baud rate
ser = serial.Serial('/dev/ttyS0', baudrate=460800)

# Set non-blocking mode (optional, but recommended for non-blocking reads)
ser.timeout = 0  # No timeout for read operations

# GPS coordinates initialization
lat = 0.000000
lon = 0.000000

#Pattern to extract the coordinates
pattern = r'lat=([\d\.]+) lon=([\d\.]+)'

print("Start of reception")
while True:
    if ser.in_waiting > 0:
        try:
            # Read data from the serial port and decode it to UTF-8 with leading/trailing whitespace removed
            data = ser.readline().decode('utf-8').strip()
            print(data)
            
            # Check if the message level is [INFO]
            if "[INFO]" in data:
                # Check if the message contains the GPS coordinates
                if "task_process_gnss_data" in data:
                    # Use regular expression to extract latitude and longitude values
                    matches = re.findall(pattern, data)
                    if matches:
                        # Convert matched strings to floats for latitude and longitude
                        lat = float(matches[0][0])
                        lon = float(matches[0][1])
                        print("Latitude :", lat)
                        print("Longitude :", lon)

            # Check if the message level is [TRACE]
            elif "[TRACE]" in data:
                pass

            # Check if the message level is [ERROR]
            elif "[ERROR]" in data:
                pass
                
        except UnicodeDecodeError:
            print("Error: Invalid character received (UnicodeDecodeError)")

