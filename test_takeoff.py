#region import 
import time
import pymavlink.mavutil as utility
import pymavlink.dialects.v20.all as dialect
#endregion

#region initialisation

# takeoff altitude definition
TAKEOFF_ALTITUDE = 20

# arm disarm definitions
VEHICLE_ARM = 1
VEHICLE_DISARM = 0

#region desired flight mode
FLIGHT_MODE = "GUIDED"
flight_modes = vehicle.mode_mapping()

# check the desired flight mode is supported
if FLIGHT_MODE not in flight_modes.keys():

    # inform user that desired flight mode is not supported by the vehicle
    print(FLIGHT_MODE, "is not supported")

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
set_mode_message = dialect.MAVLink_command_long_message(
    target_system=vehicle.target_system,
    target_component=vehicle.target_component,
    command=dialect.MAV_CMD_DO_SET_MODE,
    confirmation=0,
    param1=dialect.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    param2=flight_modes[FLIGHT_MODE],
    param3=0,
    param4=0,
    param5=0,
    param6=0,
    param7=0
)

#create takeoff command
takeoff_command = dialect.MAVLink_command_long_message(
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
land_command = dialect.MAVLink_command_long_message(
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

#endregion

#region connect to vehicle
vehicle = utility.mavlink_connection(device="127.0.0.1:14551")

# wait for a heartbeat
vehicle.wait_heartbeat()

# inform user
print("Connected to system:", vehicle.target_system, ", component:", vehicle.target_component)
#endregion

#region change flight mode, si réussi modeok = True 
# change flight mode
vehicle.mav.send(set_mode_message)

#variable pour stopper le programme en cas d'échec  
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
            print("Changing mode to", FLIGHT_MODE, "accepted from the vehicle")
            modeok = True

        # not accepted
        else:

            # inform the user
            print("Changing mode to", FLIGHT_MODE, "failed")
            modeok = False

        # break the loop
        break
#endregion

#region arm vehicle, si réussi arm = True 

#variable pour stopper le programme en cas d'échec        
arm = False 

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
            arm = True

        else:

            # print that vehicle is not armed
            print("Vehicle is not armed!")
            arm = False
        
        break

#endregion

#region takeoff wait land 

#on fait ça que si les précédent ont réussis
if arm:
    # takeoff the vehicle
    vehicle.mav.send(takeoff_command)

    # inform user
    print("Sent takeoff command to vehicle")

    # check the pre-arm
    while True:

            # catch GLOBAL_POSITION_INT message
            message = vehicle.recv_match(type=dialect.MAVLink_global_position_int_message.msgname, blocking=True)

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

    # wait 10 seconds
    print("Waiting 10 seconds")
    time.sleep(10)

    # land the vehicle
    vehicle.mav.send(land_command)

    # inform user
    print("Sent land command to vehicle")

    # check the pre-arm
    while True:

        # catch GLOBAL_POSITION_INT message
        message = vehicle.recv_match(type=dialect.MAVLink_global_position_int_message.msgname, blocking=True)

        #convert message to dictionary
        message = message.to_dict()

        # get relative altitude
        relative_altitude = message["relative_alt"] * 1e-3

        #print out the message
        print("Relative Altitude", relative_altitude, "meters")

        # check if reached the target altitude
        if relative_altitude < 1:

            # break the loop
            break

    # wait some seconds to land
    time.sleep(10)

    # print out that land is successful
    print("Landed successfully")

#endregion

print("Fin du programme test_takeoff.py")