#region import 
import time
import pymavlink.mavutil as utility
import pymavlink.dialects.v20.all as dialect
#endregion

#region connect to vehicle
vehicle = utility.mavlink_connection(device="127.0.0.1:14551")

# wait for a heartbeat
vehicle.wait_heartbeat()

# inform user
print("Connected to system:", vehicle.target_system, ", component:", vehicle.target_component)
#endregion

#region desired flight mode
GUIDED_FLIGHT_MODE = "GUIDED"
AUTO_FLIGHT_MODE = "AUTO"
RTL_FLIGHT_MODE = "RTL"
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

#endregion

#region messages mavlink 

#create arm message
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

# create mission count message
message = dialect.MAVLink_mission_count_message(target_system=vehicle.target_system,
                                                target_component=vehicle.target_component,
                                                count=len(target_locations) + 2,
                                                mission_type=dialect.MAV_MISSION_TYPE_MISSION)

#endregion

#region initialisation

# takeoff altitude definition
TAKEOFF_ALTITUDE = 20

# arm disarm definitions
VEHICLE_ARM = 1
VEHICLE_DISARM = 0

#endregion

'''

L'objectif ici va être d'envoyer au RSV la mission que l'on souhaite, d'armer le véhicule et de passer en mode auto. 

Une fois la mission fini on veut effectuer un RTL

'''

# create mission item list
target_locations = ((43.058248, 6.130403, 10),
                    (43.053231, 6.126283, 10),
                    (43.049719, 6.128, 10))

#region arm vehicle

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

#region détection de fin de mission et RTL



#endregion

print("Fin du programme test_takeoff.py")