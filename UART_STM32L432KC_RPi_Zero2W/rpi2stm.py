import serial
import time
import random
import re

# Set the serial port and baud rate
ser = serial.Serial('/dev/ttyS0', baudrate=9600)

# Set non-blocking mode (optional, but recommended for non-blocking reads)
ser.timeout = 0  # No timeout for read operations

data = ""  # Initialize an empty string to store the data
flag = False  # Flag to indicate if data collection is active


def receive_data():
    """Receives and processes incoming data from the serial port"""
    global data, flag  # Access global variables inside the function

    if ser.in_waiting > 0:
        try:
            char = ser.read(1).decode('utf-8')

            if char == "$":
                flag = True
                data = ""  # Clear the data string

            if flag:
                if char != "$" and char != "\n":
                    data += char

                if char == '\n' and data[-1] == '\r' :
                    flag = False
                    
        except UnicodeDecodeError:
            print("Error: Invalid character received (UnicodeDecodeError)")
            flag = False  # Optionally reset the flag



def send_depth2reach(depth):
    """Sends the depth to reach"""
    try:
        # Format the message using string formatting
        message = f"$DS;{depth:0>6.2f}\r\n"

        # Convert the message to bytes before sending
        message_bytes = message.encode('utf-8')
        ser.write(message_bytes)
        print(f"TX :{message}")

    except Exception as e:
        print(f"Error sending data: {e}")


def extract_number(data_string):
    """Extracts the numerical value after the semicolon (;) from a string."""
    if not data_string:
        return None  # Handle empty string case

    try:
        # Split the string at the semicolon
        parts = data_string.split(";")

        # Check if there are at least two parts (semicolon-separated)
        if len(parts) < 2:
            return None  # Invalid format

        # Extract and return the numerical value after the semicolon
        value_str = parts[1]
        return float(value_str)

    except ValueError:
        # Handle non-numerical values
        return None

last_send_time = 0  # Initialize a timestamp for tracking last send time
current_depth = 0.0         # Initialize depth

while True:
    # Receive data continuously
    receive_data()

    # Check the incoming message's ID
    if data[0:2] == "DC" and not flag:
        new_current_depth = extract_number(data)
        if new_current_depth != current_depth  and new_current_depth is not None:
            current_depth = new_current_depth
            print(f"Current depth = {current_depth:06.2f}")
    
    # Get the current time
    current_time = time.time()

    # Check if 10 seconds have elapsed since the last send_data call
    if current_time - last_send_time >= 10:

        # Generate random float for depth to reach within the specified range
        target_depth = random.uniform(0,200.0)
        
        # Send data every 10 seconds
        send_depth2reach(target_depth)

        # Update the last_send_time to the current time
        last_send_time = current_time