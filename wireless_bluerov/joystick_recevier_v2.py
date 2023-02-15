# joystick_receiver.py
import serial
import struct

# Open the serial connection
ser = serial.Serial(port='COM6', baudrate=115200, timeout=1)

# Receive joystick data over the serial connection
def receive_joystick_data():
    # Read the data from the serial connection
    data = ser.read(16)

    # Unpack the binary string into joystick data
    x, y, z, yaw = struct.unpack('<ffff', data)

    # Return the joystick data
    return x, y, z, yaw

# Continuously receive and print joystick data
while True:
    x, y, z, yaw = receive_joystick_data()
    print('x:', x)
    print('y:', y)
    print('z:', z)
    print('yaw:', yaw)

# Close the serial connection when the program exits
ser.close()
