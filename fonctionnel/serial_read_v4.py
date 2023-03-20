import time 
import serial
import struct
import pymavlink.mavutil as utility
import pymavlink.dialects.v20.all as dialect
from math import pi, sin, cos, sqrt, atan2

#region reception des données
def from_8_bits_to_PWM(val):
    val +=127
    val /= 254
    val += 1
    val *= 1000
    return val 

#format de réception des données
fmt = "!bb"

#ouverture du port serie
ser = serial.Serial(port='COM3',
		    baudrate = 300,
		    parity = serial.PARITY_NONE,
		    stopbits=serial.STOPBITS_ONE,
		    bytesize=serial.EIGHTBITS,
		    timeout=1)
#endregion

#permet de créer le messsage mavlink qui ecrie sur les rc 
def override_channels(vehicle, channels):
    temp_channels = [65535] * 18
    for i in range(len(temp_channels)):
        if i + 1 in channels.keys():
            temp_channels[i] = channels[i + 1]
    channels = temp_channels
    return dialect.MAVLink_rc_channels_override_message(vehicle.target_system, vehicle.target_component, *channels)

#assure que le pwm envoyé sera entre 1000 et 2000
def safe_pwm(value, offset):
    value = value + offset
    if value < 1000:
        value = 1000
    if value > 2000:
        value = 2000
    return value

#region connect to vehicle
print('tentative de connexion')
vehicle = utility.mavlink_connection(device="tcp:127.0.0.1:5762")

# wait for a heartbeat
vehicle.wait_heartbeat()

# inform user
print("Connected to system:", vehicle.target_system, ", component:", vehicle.target_component)
#endregion

#region desired flight mode
GUIDED_FLIGHT_MODE = "GUIDED"
AUTO_FLIGHT_MODE = "AUTO"
#RTL_FLIGHT_MODE = "RTL"
MANUAL_FLIGHT_MODE = "MANUAL"
flight_modes = vehicle.mode_mapping()

# check the desired flights modes are supported
if GUIDED_FLIGHT_MODE not in flight_modes.keys():

    # inform user that desired flight mode is not supported by the vehicle
    print(GUIDED_FLIGHT_MODE, "is not supported")

    # exit the code
    exit(1)

if AUTO_FLIGHT_MODE not in flight_modes.keys():

    # inform user that desired flight mode is not supported by the vehicle
    print(AUTO_FLIGHT_MODE, "is not supported")

    # exit the code
    exit(1)

#if RTL_FLIGHT_MODE not in flight_modes.keys():

    # inform user that desired flight mode is not supported by the vehicle
 #   print(RTL_FLIGHT_MODE, "is not supported")

    # exit the code
  #  exit(1)

if MANUAL_FLIGHT_MODE not in flight_modes.keys():

    # inform user that desired flight mode is not supported by the vehicle
    print(MANUAL_FLIGHT_MODE, "is not supported")

    # exit the code
    exit(1)

#endregion

#region messages mavlink 

#create arm message
VEHICLE_ARM = 1
vehicle_arm_message = dialect.MAVLink_command_long_message(
    target_system=vehicle.target_system,
    target_component=vehicle.target_component,
    command=dialect.MAV_CMD_COMPONENT_ARM_DISARM,
    confirmation=0,
    param1=VEHICLE_ARM,
    param2=0,
    param3=0,
    param4=0,
    param5=0,
    param6=0,
    param7=0
)

#create disarm message
VEHICLE_DISARM = 0
vehicle_disarm_message = dialect.MAVLink_command_long_message(
    target_system=vehicle.target_system,
    target_component=vehicle.target_component,
    command=dialect.MAV_CMD_COMPONENT_ARM_DISARM,
    confirmation=0,
    param1=VEHICLE_DISARM,
    param2=0,
    param3=0,
    param4=0,
    param5=0,
    param6=0,
    param7=0
)

# create change mode message
set_guided_mode_message = dialect.MAVLink_command_long_message(
    target_system=vehicle.target_system,
    target_component=vehicle.target_component,
    command=dialect.MAV_CMD_DO_SET_MODE,
    confirmation=0,
    param1=dialect.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    param2=flight_modes[GUIDED_FLIGHT_MODE],
    param3=0,
    param4=0,
    param5=0,
    param6=0,
    param7=0
)
set_auto_mode_message = dialect.MAVLink_command_long_message(
    target_system=vehicle.target_system,
    target_component=vehicle.target_component,
    command=dialect.MAV_CMD_DO_SET_MODE,
    confirmation=0,
    param1=dialect.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    param2=flight_modes[AUTO_FLIGHT_MODE],
    param3=0,
    param4=0,
    param5=0,
    param6=0,
    param7=0
)
#set_rtl_mode_message = dialect.MAVLink_command_long_message(
#    target_system=vehicle.target_system,
#    target_component=vehicle.target_component,
#    command=dialect.MAV_CMD_DO_SET_MODE,
#    confirmation=0,
#    param1=dialect.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
#    param2=flight_modes[RTL_FLIGHT_MODE],
#    param3=0,
#    param4=0,
#    param5=0,
#    param6=0,
#    param7=0
#)
set_manual_mode_message = dialect.MAVLink_command_long_message(
    target_system=vehicle.target_system,
    target_component=vehicle.target_component,
    command=dialect.MAV_CMD_DO_SET_MODE,
    confirmation=0,
    param1=dialect.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    param2=flight_modes[MANUAL_FLIGHT_MODE],
    param3=0,
    param4=0,
    param5=0,
    param6=0,
    param7=0
)

#endregion

print('start reading orders')
data=[0,0,0,0,0,0,0,0]

while True :
    try :
        #region reception d'ordre radio
        data_brut = ser.read(struct.calcsize(fmt))
        number, value = struct.unpack(fmt, data_brut)
        data[number]=value
        #print(*data)
        converted_data = [int(from_8_bits_to_PWM(x)) for x in data]
        #print("RC 1 : " + str(converted_data[0]) + "\tRC3 :" + str(converted_data[3]))
        #endregion 
        
        channels = {1: converted_data[0], 2: 1500, 3: converted_data[3], 4: 1500}
        print('envoi' + str(channels))
        
        # create rc channels override message
        message = override_channels(vehicle=vehicle, channels=channels)

        # send the message to the vehicle
        vehicle.mav.send(message)
        
        if data[4] == 1 : 
            print('Passage en auto')
            #region change to auto flight mode
            vehicle.mav.send(set_auto_mode_message)

            # do below always after changing flight mode
            while True:

                # catch COMMAND_ACK message
                message = vehicle.recv_match(type=dialect.MAVLink_command_ack_message.msgname, blocking=True)

                # convert this message to dictionary
                message = message.to_dict()

                # check is the COMMAND_ACK is for DO_SET_MODE
                if message["command"] == dialect.MAV_CMD_DO_SET_MODE:

                    # check the command is accepted or not
                    if message["result"] == dialect.MAV_RESULT_ACCEPTED:

                        # inform the user
                        print("Changing mode to", AUTO_FLIGHT_MODE, "accepted from the vehicle")

                    # not accepted
                    else:

                        # inform the user
                        print("Changing mode to", AUTO_FLIGHT_MODE, "failed")

                    # break the loop
                    break

            #endregion

        if data[5] == 1 : 
            print('Passage en RTL')
            #region change flight mode
            vehicle.mav.send(set_rtl_mode_message)

            # do below always after changing flight mode
            while True:

                # catch COMMAND_ACK message
                message = vehicle.recv_match(type=dialect.MAVLink_command_ack_message.msgname, blocking=True)

                # convert this message to dictionary
                message = message.to_dict()

                # check is the COMMAND_ACK is for DO_SET_MODE
                if message["command"] == dialect.MAV_CMD_DO_SET_MODE:

                    # check the command is accepted or not
                    if message["result"] == dialect.MAV_RESULT_ACCEPTED:

                        # inform the user
                        print("Changing mode to", RTL_FLIGHT_MODE, "accepted from the vehicle")

                    # not accepted
                    else:

                        # inform the user
                        print("Changing mode to", RTL_FLIGHT_MODE, "failed")

                    # break the loop
                    break

            #endregion

        if data[6] == 1 : 
            print('Passage en Manuel')
            #region change flight mode
            vehicle.mav.send(set_manual_mode_message)

            # do below always after changing flight mode
            while True:

                # catch COMMAND_ACK message
                message = vehicle.recv_match(type=dialect.MAVLink_command_ack_message.msgname, blocking=True)

                # convert this message to dictionary
                message = message.to_dict()

                # check is the COMMAND_ACK is for DO_SET_MODE
                if message["command"] == dialect.MAV_CMD_DO_SET_MODE:

                    # check the command is accepted or not
                    if message["result"] == dialect.MAV_RESULT_ACCEPTED:

                        # inform the user
                        print("Changing mode to", MANUAL_FLIGHT_MODE, "accepted from the vehicle")

                    # not accepted
                    else:

                        # inform the user
                        print("Changing mode to", MANUAL_FLIGHT_MODE, "failed")

                    # break the loop
                    break

            #endregion
    except :
        print ('Rien reçu') 
