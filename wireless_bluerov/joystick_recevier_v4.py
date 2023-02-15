import serial
import struct
import sys
import time
from threading import Thread

# Set up the serial connection to the COM port
ser = serial.Serial("COM5", baudrate=9600, timeout=1)

# Define the format for the data to be received
fmt = "!iiiiiiii"

# Function to receive and unpack the data
print('debut while')
def receive_data():
    while True:
        print('boucle')
        # Read the binary data from the serial connection
        binary_data = ser.read(struct.calcsize(fmt))
        print('binary data length : ', len(binary_data))
        expected_length = struct.calcsize(fmt)
        print('expected length:', expected_length)
        
        try:
            data = struct.unpack(fmt, binary_data)
            print(data)
        except:
            print('error unpacking data')

# Start the receiving thread
print('debut thread')
receive_thread = Thread(target=receive_data)
receive_thread.start()

try :
    while True :
        time.sleep(0.1)
except KeyboardInterrupt:
    print ("Exiting program")
    ser.close()
    sys.exit()

