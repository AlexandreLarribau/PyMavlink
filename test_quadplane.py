#region import 
import time
import pymavlink.mavutil as utility
import pymavlink.dialects.v20.all as dialect
from math import pi, sin, cos, sqrt, atan2
#endregion

#region connect to vehicle
vehicle = utility.mavlink_connection(device="127.0.0.1:14551")

# wait for a heartbeat
vehicle.wait_heartbeat()

# inform user
print("Connected to system:", vehicle.target_system, ", component:", vehicle.target_component)
#endregion

# create mission item list (put before the mavlink message)
ma_mission = (  (43.05443038, 6.12620687, 5),
                (43.06708954, 6.12620687, 10),
                (43.06708954, 6.11012366, 15),
                (43.05443038, 6.11012366, 10),
                (43.05443038, 6.12620687, 5)   )

#region flights modes
GUIDED_FLIGHT_MODE = "GUIDED"
AUTO_FLIGHT_MODE = "AUTO"
RTL_FLIGHT_MODE = "RTL"
QLAND_FLIGHT_MODE = "QLAND"
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

if RTL_FLIGHT_MODE not in flight_modes.keys():

    # inform user that desired flight mode is not supported by the vehicle
    print(RTL_FLIGHT_MODE, "is not supported")

    # exit the code
    exit(1)

if QLAND_FLIGHT_MODE not in flight_modes.keys():

    # inform user that desired flight mode is not supported by the vehicle
    print(QLAND_FLIGHT_MODE, "is not supported")

    # exit the code
    exit(1)

#endregion

#region messages mavlink 

# arm disarm definitions
VEHICLE_ARM = 1
VEHICLE_DISARM = 0

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
set_rtl_mode_message = dialect.MAVLink_command_long_message(
    target_system=vehicle.target_system,
    target_component=vehicle.target_component,
    command=dialect.MAV_CMD_DO_SET_MODE,
    confirmation=0,
    param1=dialect.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    param2=flight_modes[RTL_FLIGHT_MODE],
    param3=0,
    param4=0,
    param5=0,
    param6=0,
    param7=0
)
set_qland_mode_message = dialect.MAVLink_command_long_message(
    target_system=vehicle.target_system,
    target_component=vehicle.target_component,
    command=dialect.MAV_CMD_DO_SET_MODE,
    confirmation=0,
    param1=dialect.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    param2=flight_modes[QLAND_FLIGHT_MODE],
    param3=0,
    param4=0,
    param5=0,
    param6=0,
    param7=0
)

# create mission count message
message = dialect.MAVLink_mission_count_message(target_system=vehicle.target_system,
                                                target_component=vehicle.target_component,
                                                count=len(ma_mission) + 2,
                                                mission_type=dialect.MAV_MISSION_TYPE_MISSION)

#create takeoff command
TAKEOFF_ALTITUDE = 5
takeoff_message = dialect.MAVLink_command_long_message(
            target_system=vehicle.target_system,
            target_component=vehicle.target_component,
            command=dialect.MAV_CMD_NAV_TAKEOFF,
            confirmation=0,
            param1=0,
            param2=0,
            param3=0,
            param4=0,
            param5=0,
            param6=0,
            param7=TAKEOFF_ALTITUDE
            )

#create land command
land_message = dialect.MAVLink_command_long_message(
            target_system=vehicle.target_system,
            target_component=vehicle.target_component,
            command=dialect.MAV_CMD_NAV_LAND,
            confirmation=0,
            param1=0,
            param2=0,
            param3=0,
            param4=0,
            param5=0,
            param6=0,
            param7=0
            )

#endregion

#region sending mission to vehicle
vehicle.mav.send(message)

# this loop will run until receive a valid MISSION_ACK message
while True:

    # catch a message
    message = vehicle.recv_match(blocking=True)

    # convert this message to dictionary
    message = message.to_dict()

    # check this message is MISSION_REQUEST
    if message["mavpackettype"] == dialect.MAVLink_mission_request_message.msgname:

        # check this request is for mission items
        if message["mission_type"] == dialect.MAV_MISSION_TYPE_MISSION:

            # get the sequence number of requested mission item
            seq = message["seq"]

            # create mission item int message
            if seq == 0:
                # create mission item int message that contains the home location (0th mission item)
                message = dialect.MAVLink_mission_item_int_message(target_system=vehicle.target_system,
                        target_component=vehicle.target_component,
                        seq=seq,
                        frame=dialect.MAV_FRAME_GLOBAL,
                        command=dialect.MAV_CMD_NAV_WAYPOINT,
                        current=0,
                        autocontinue=0,
                        param1=0,
                        param2=0,
                        param3=0,
                        param4=0,
                        x=0,
                        y=0,
                        z=0,
                        mission_type=dialect.MAV_MISSION_TYPE_MISSION)

            # send target locations to the vehicle
            else:

                # create mission item int message that contains a target location
                message = dialect.MAVLink_mission_item_int_message(target_system=vehicle.target_system,
                        target_component=vehicle.target_component,
                        seq=seq,
                        frame=dialect.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                        command=dialect.MAV_CMD_NAV_WAYPOINT,
                        current=0,
                        autocontinue=0,
                        param1=0,
                        param2=0,
                        param3=0,
                        param4=0,
                        x=int(ma_mission[seq - 2][0] * 1e7),
                        y=int(ma_mission[seq - 2][1] * 1e7),
                        z=ma_mission[seq - 2][2],
                        mission_type=dialect.MAV_MISSION_TYPE_MISSION)

            # send the mission item int message to the vehicle
            vehicle.mav.send(message)

    # check this message is MISSION_ACK
    elif message["mavpackettype"] == dialect.MAVLink_mission_ack_message.msgname:

        # check this acknowledgement is for mission and it is accepted
        if message["mission_type"] == dialect.MAV_MISSION_TYPE_MISSION and \
                message["type"] == dialect.MAV_MISSION_ACCEPTED:
            # break the loop since the upload is successful
            print("Mission upload is successful")
            break

#endregion

''' L'objectif ici va être de passer en guider, d'envoyer une mission, de décoller et de faire la mission. 

Une fois la mission finie on veut effectuer un RTL et un land '''

#region change flight mode into guided
vehicle.mav.send(set_guided_mode_message)
modeok = False
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
            print("Changing mode to", GUIDED_FLIGHT_MODE, "accepted from the vehicle")
            modeok = True

        # not accepted
        else:

            # inform the user
            print("Changing mode to", GUIDED_FLIGHT_MODE, "failed")
            modeok = False

        # break the loop
        break

#endregion

#region arm vehicle

#variable pour stopper le programme en cas d'échec   

if modeok:
    # check the pre-arm
    while True:

        # observe the SYS_STATUS messages
        message = vehicle.recv_match(type=dialect.MAVLink_sys_status_message.msgname, blocking=True)

        # convert to dictionary
        message = message.to_dict()

        # get sensor health
        onboard_control_sensors_health = message["onboard_control_sensors_health"]

        # get pre-arm healthy bit
        prearm_status_bit = onboard_control_sensors_health & dialect.MAV_SYS_STATUS_PREARM_CHECK
        prearm_status = prearm_status_bit == dialect.MAV_SYS_STATUS_PREARM_CHECK

        # check prearm
        if prearm_status:

            # vehicle can be armable
            print("Vehicle is armable")

            # break the prearm check loop
            break

    #arm        
    while True:
        # arm the vehicle
        print("Vehicle is arming...")

        # send arm message
        vehicle.mav.send(vehicle_arm_message)

        # wait COMMAND_ACK message
        message = vehicle.recv_match(type=dialect.MAVLink_command_ack_message.msgname, blocking=True)

        # convert the message to dictionary
        message = message.to_dict()

        # check if the vehicle is armed
        if message["result"] == dialect.MAV_RESULT_ACCEPTED and message["command"] == dialect.MAV_CMD_COMPONENT_ARM_DISARM:

            # print that vehicle is armed
            print("Vehicle is armed!")

        else:

            # print that vehicle is not armed
            print("Vehicle is not armed!")
        
        break

#endregion

#region takeoff

#on fait ça que si les précédent ont réussis
# takeoff the vehicle
vehicle.mav.send(takeoff_message)

# inform user
print("Sent takeoff command to vehicle")

# check the pre-arm
while True:

    # catch GLOBAL_POSITION_INT message
    message = vehicle.recv_match(type=dialect.MAVLink_global_position_int_message.msgname,
                                blocking=True)

    # convert message to dictionary
    message = message.to_dict()

    # get relative altitude
    relative_altitude = message["relative_alt"] * 1e-3

    # print out the message
    print("Relative Altitude", relative_altitude, "meters")

    # check if reached the target altitude
    if TAKEOFF_ALTITUDE - relative_altitude < 1:

        # print out that takeoff is successful
        print("Takeoff to", TAKEOFF_ALTITUDE, "meters is successful")

        # break the loop
        break

    time.sleep(1)


#endregion

#region change flight mode
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

#region on attend d'avoir fini la mission pour land 
current_mode = vehicle.recv_match(type='HEARTBEAT', blocking=True).custom_mode
print("mode : ", current_mode)

while current_mode == 10:
    time.sleep(2)
    print("mission en cours")
    current_mode = vehicle.recv_match(type='HEARTBEAT', blocking=True).custom_mode

#endregion

#region land

# wait 10 seconds
print("Waiting 5 seconds")
time.sleep(5)

# land the vehicle

vehicle.mav.send(set_qland_mode_message)
modeok = False
# do below always after changing flight mode
while not modeok:

    # catch COMMAND_ACK message
    message = vehicle.recv_match(type=dialect.MAVLink_command_ack_message.msgname, blocking=True)

    # convert this message to dictionary
    message = message.to_dict()

    # check is the COMMAND_ACK is for DO_SET_MODE
    if message["command"] == dialect.MAV_CMD_DO_SET_MODE:

        # check the command is accepted or not
        if message["result"] == dialect.MAV_RESULT_ACCEPTED:

            # inform the user
            print("Changing mode to", QLAND_FLIGHT_MODE, "accepted from the vehicle")
            modeok = True

        # not accepted
        else:

            # inform the user
            print("Changing mode to", QLAND_FLIGHT_MODE, "failed")
            modeok = False

# print out that land is successful
print("Landed successfully")

#endregion

print("Fin du programme test_quadplane.py")