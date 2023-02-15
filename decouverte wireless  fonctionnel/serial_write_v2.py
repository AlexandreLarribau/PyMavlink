import time 
import serial
import struct
import pygame

ser = serial.Serial(port='/dev/ttyUSB0', 
		    baudrate="9600",
		    parity=serial.PARITY_NONE,
		    stopbits=serial.STOPBITS_ONE,
		    bytesize=serial.EIGHTBITS,
		    timeout=1)

def limit(val, min, max):
    """ Clamp value between min/max values """
    if val > max:
        return max
    if val < min:
        return min
    return val


def joy_to_byte(val):
    """" Convert float -1 to 1 to byte (-127:127) """
    adjusted = int((val * 255 -1)/2)
    return limit(adjusted, -127, 127)

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

# Define the format for the data to be sent
fmt = "!bbbbbbbb"

# Initialize the data with 0
data = [0, 0, 0, 0, 0, 0, 0, 0]
prev_data = [0, 0, 0, 0, 0, 0, 0, 0]
packets_sent = 1

while True:
    # Read events from the joystick
    for event in pygame.event.get():
        if event.type == pygame.JOYAXISMOTION:
            # Update the data array based on joystick axis values
            for i in range (4):
                data[i] = joy_to_byte(joystick.get_axis(i))
        
        if event.type == pygame.JOYBUTTONUP or event.type == pygame.JOYBUTTONDOWN:
            for i in range (4):
                data[4+i] = joystick.get_button(i)
            

    # noticing change in joystick data
    change=False
    for i in range (8):
        if data[i] != prev_data[i]:
            change = True
            prev_data[i] =data[i]
            
    #sending data         
    if change : 
        print('Sending Packet n°' + str(packets_sent))
        print (*data)
        ser.write(struct.pack(fmt, *data))
        packets_sent +=1
        time.sleep(0.1) # add a small delay to avoid flooding the serial port

    
    

