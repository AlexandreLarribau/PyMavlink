import pygame
import socket

# initialize pygame and the joystick
pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

# create a socket connection to MAVProxy
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(('COM6'))


print('Connexion réussie : envoi en cours')

# main loop to read the joystick and send commands to MAVProxy
while True:
    pygame.event.get()
    x_axis = joystick.get_axis(0)
    y_axis = joystick.get_axis(1)
    z_axis = joystick.get_axis(2)
    command = f"rc 3 {1500 + 500 * x_axis} rc 4 {1500 + 500 * y_axis} rc 5 {1500 + 500 * z_axis}\n"
    sock.send(command.encode())
