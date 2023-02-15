"""
Wireless BlueROV2 control.

This script reads the XBox controller and updates the modem queue to always have the latest
joystick positions, so that when it is sent we are sure it is always the latest values.

To read the joystick we use:
https://github.com/FRC4564/Xbox

Download the xbox controller script with:
wget https://raw.githubusercontent.com/FRC4564/Xbox/master/xbox.py

"""
import time
import serial
import struct
import xbox

ser = serial.Serial(port='COM5', baudrate=300, timeout=1)

#region conversion joystick
def button_to_byte(tpl):
    """ Convert button tuple to byte """
    return sum([v<<k for k,v in enumerate(tpl)])

def clamp(val, min, max):
    """ Clamp value between min/max values """
    if val > max:
        return max
    if val < min:
        return min
    return val

def stick_to_byte(val):
    """" Convert float -1 to 1 to byte (0-255) """
    adjusted = int((val+1.0)/2.0 * 255)
    return clamp(adjusted, 0, 255)
#endregion

def run(joy):
    print("Starting to send joystick data")
    while True:
        #region read data from joystick
        sticks = (joy.leftX(), joy.leftY(), joy.rightX(), joy.rightY())

        stickbytes = [stick_to_byte(x) for x in sticks]

        pads1 = (joy.dpadUp(), joy.dpadRight(), joy.dpadDown(), joy.dpadLeft(), joy.A(), joy.B(), joy.X(), joy.Y())
        pads2 = (joy.Start(), joy.Back())

        pads1byte = button_to_byte(pads1)
        pads2byte = button_to_byte(pads2)

        all_bytes = stickbytes
        all_bytes.extend([pads1byte, pads2byte])
        print("Sending values: {}".format(all_bytes))
        data_to_send = struct.pack("BBBBxxBB", *all_bytes)
        #endregion 

        #region send it 
        ser.cmd_flush_queue()
        ser.cmd_queue_packet(data_to_send)
        #endregion

        time.sleep(0.05)

def main():
    
    joy  = xbox.Joystick()

    try:
        run(joy)
    except KeyboardInterrupt:
        pass
    finally:
        joy.close()

if __name__ == "__main__":
    main()
