import time 
import serial
import struct

fmt = "!bbbbbbbb"

ser = serial.Serial(port='/dev/ttyUSB1',
		    baudrate = 9600,
		    parity = serial.PARITY_NONE,
		    stopbits=serial.STOPBITS_ONE,
		    bytesize=serial.EIGHTBITS,
		    timeout=1)

print('start reading')
while True :
    try :
        data_brut = ser.read(struct.calcsize(fmt))
        joy0, joy1 ,joy2 ,joy3 ,but0 ,but1 ,but2 ,but3 = struct.unpack(fmt, data_brut)
        print("joy0 =" + str(joy0)
              + "\tjoy1 =" + str(joy1)
              + "\tjoy2 =" + str(joy2)
              + "\tjoy3 =" + str(joy3)
              + "\tbut0 =" + str(but0)
              + "\tbut1 =" + str(but1)
              + "\tbut2 =" + str(but2)
              + "\tbut3 =" + str(but3) + "\n")
    except :
        
        print ('Rien reçu') 