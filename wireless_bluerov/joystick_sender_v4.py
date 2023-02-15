import pygame
import serial
import struct
import sys
import time
from queue import Queue

# Initialize pygame
pygame.init()

# Define the number of joysticks
joystick_count = pygame.joystick.get_count()

# Check if there is a joystick available
if joystick_count == 0:
    print("No joystick found.")
    sys.exit()

# Select the first joystick available
joystick = pygame.joystick.Joystick(0)
joystick.init()

# Set up the serial connection to the COM port
ser = serial.Serial("/dev/ttyUSB0", baudrate=115200, timeout=3)

# Define the format for the data to be sent
fmt = "!iiiiiiii"

# Initialize the data with 0
data = [0, 0, 0, 0, 0, 0, 0, 0]

# Create a queue to hold the data to be sent
send_queue = Queue()

# Function to send the data over the serial connection
def send_data(data, send_queue):
    while True:
        # Check if there is data in the queue
        if not send_queue.empty():
            # Get the data from the queue
            data_to_send = send_queue.get()
            # Pack the data into a binary string using the format defined in "fmt"
            binary_data = struct.pack(fmt, *data_to_send)
            # Write the binary data to the serial connection
            ser.write(binary_data)

# Main loop to read the joystick and send the data
while True:
    # Check for events
    for event in pygame.event.get():
        # Check if the joystick has moved
        if event.type == pygame.JOYAXISMOTION:
            # Get the value of the joystick axis
            axis = round(joystick.get_axis(event.axis))
            # Update the data with the new value
            data[event.axis] = axis
            # Put the data in the queue to be sent
            send_queue.put(data)
            print('sending :', data )

# Start the sending thread
send_thread = Thread(target=send_data, args=(data, send_queue))
send_thread.start()

# Clean up the pygame
pygame.quit()
