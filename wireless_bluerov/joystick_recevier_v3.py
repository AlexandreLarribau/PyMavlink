"""
Wireless BlueROV2 control.

This is the receiver which decodes the message from the Water Linked modem and
sends MavLink messages to control a BlueROV2.

"""
from __future__ import print_function, division
import time
import struct
import sys
import serial

ser = serial.Serial(port='COM5', baudrate=300, timeout=1)



def byte_to_button(val):
    """ Convert button byte to list of values """
    btn = []
    for x in range(8):
        valid = (val >> x) & 0x01
        if valid > 0:
            btn.append(True)
        else:
            btn.append(False)

    return btn

def byte_to_pwm(bytevalue):
    """
    Convert bytevalue 0-255 to PWM

    Return PWM in range 1100-1900
    """
    centered = bytevalue - 127.0  # Range: -127 - 127
    gained = centered * 0.25
    scaled = gained / 127 * 400 # Range: -400 - 400
    return int(scaled) + 1500 #  Range: 1100 - 1900

def run(): 
    print("Waiting for modem packets")
    timeout_max = 5
    timeout_cnt = 0
    while True:
        pkt = ser.read(8)
        if pkt:
            timeout_cnt = timeout_max
            #print("Got data: {}".format(pkt))
            joystick = struct.unpack("BBBBxxBB", pkt)
            #print("Got joystick data {}".format(joystick))
            leftX, leftY, rightX, rightY, b_pads1, b_pads2 = joystick
            #print(leftX, leftY, rightX, rightY)
            pads1 = byte_to_button(b_pads1)
            pads2 = byte_to_button(b_pads2)
            #print(pads1, pads2)

            #on verra ca plus tard
            

            # dup, dright, ddown, dleft, a, b, x, y = pads1
            _, _, _, _, _, b, x, y = pads1
            if b:
                print('ordre mode manuel')
                #pix.change_mode("MANUAL")
            if x:
                print('ordre mode alt hold')
                #pix.change_mode("ALT_HOLD")
            if y:
                print('ordre mode stabilize')
                #pix.change_mode("STABILIZE")

            #pix.set_rc_channel_pwm(4, byte_to_pwm(leftX))
            #pix.set_rc_channel_pwm(3, byte_to_pwm(leftY))
            #pix.set_rc_channel_pwm(6, byte_to_pwm(rightX))
            #pix.set_rc_channel_pwm(5, byte_to_pwm(rightY))

            arm = pads2[0]
            disarm = pads2[1]
            if arm:
                print("Arming!")
                #pix.arm()
            if disarm:
                print("Disarming!")
                #pix.disarm()
            

        else:
            #nothing received
            print('rien reçu')
            # Got no packet, lets stop movement
            #pix.set_rc_channel_pwm(4, byte_to_pwm(127))
            #pix.set_rc_channel_pwm(3, byte_to_pwm(127))
            #pix.set_rc_channel_pwm(6, byte_to_pwm(127))
            #pix.set_rc_channel_pwm(5, byte_to_pwm(127))

            timeout_cnt -= 1
            if timeout_cnt < 0:
                # Got no packet for a while, disarming
                print("Timeout, disarming")
                #pix.disarm()
                timeout_cnt = timeout_max

        #time.sleep(0.1)

def main():
    
    run()

if __name__ == "__main__":
    main()
