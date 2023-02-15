# joystick_sender.py
import serial
import struct

# Open the serial connection
ser = serial.Serial(port='COM6', baudrate=115200, timeout=1)

# Send joystick data over the serial connection
def send_joystick_data(x, y, z, yaw):
    # Pack the joystick data into a binary string
    data = struct.pack('<ffff', x, y, z, yaw)

    # Write the data to the serial connection
    ser.write(data)

# Continuously send joystick data
while True:
    x = float(input('Enter x: '))
    y = float(input('Enter y: '))
    z = float(input('Enter z: '))
    yaw = float(input('Enter yaw: '))
    send_joystick_data(x, y, z, yaw)

# Close the serial connection when the program exits
ser.close()
