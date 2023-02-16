import time 
import serial
import struct
import pygame

ser = serial.Serial(port='COM6', 
		    baudrate="300",
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
fmt = "!bb"

# Initialize the data with 0
data = [0, 0, 0, 0, 0, 0, 0, 0]
prev_data = [0, 0, 0, 0, 0, 0, 0, 0]

#on va maintenant envoyer que ce qui à changé au format numéro , valeur 
changes=[-1,0] 

packets_sent = 0
start_time = time.time() 
packets_per_second = [0]*10 
sec = 0

while True:
    # Read events from the joystick
    for event in pygame.event.get():
        if event.type == pygame.JOYAXISMOTION:
            # Update the data array based on joystick axis values
            data[0] = joy_to_byte(joystick.get_axis(0))
            data[1] = joy_to_byte(joystick.get_axis(1))
            data[2] = joy_to_byte(joystick.get_axis(3))
            data[3] = joy_to_byte(joystick.get_axis(4))
        
        if event.type == pygame.JOYBUTTONUP or event.type == pygame.JOYBUTTONDOWN:
            for i in range (4):
                data[4+i] = joystick.get_button(i)
            
    # noticing change in joystick data
    change=False
    number = -1
    for i in range (8):
        if data[i] != prev_data[i]:
            change = True
            number = i
            prev_data[i] =data[i]
            
    #sending data         
    if change :
        packets_sent +=1
        changes[0]=number
        changes[1]=data[number]
        print('Sending Packet n°' + str(packets_sent))
        print (*changes)
        ser.write(struct.pack(fmt, *changes))
        time.sleep(20/ser.baudrate)  # add a small delay to avoid flooding the serial port
                                     #see the txt document to understand why 20/ser.baudrate 

    #trying to count data each seconds 
    if (time.time() - start_time) > 1 : 
        start_time = time.time() 
        packets_per_second[sec]=packets_sent
        sec += 1
        packets_sent = 0
        if sec == 10 : 
            sec = 0 
            print ("packets sent these last ten secs : " + str(packets_per_second))
            packets_per_second = [0]*10
    
    
    

