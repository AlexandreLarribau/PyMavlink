import time 
import serial
import struct

fmt = "!bb"

ser = serial.Serial(port='/dev/ttyUSB1',
		    baudrate = 9600,
		    parity = serial.PARITY_NONE,
		    stopbits=serial.STOPBITS_ONE,
		    bytesize=serial.EIGHTBITS,
		    timeout=1)

print('start reading')
data=[0,0,0,0,0,0,0,0]
while True :
    try :
        data_brut = ser.read(struct.calcsize(fmt))
        number, value = struct.unpack(fmt, data_brut)
        data[number]=value
        print(*data)
    except :
        
        print ('Rien reçu') 