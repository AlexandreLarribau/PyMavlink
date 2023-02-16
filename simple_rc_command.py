
import pymavlink.mavutil as utility
import pymavlink.dialects.v20.all as dialect
import time 

def override_channels(vehicle, channels):
    temp_channels = [65535] * 18
    for i in range(len(temp_channels)):
        if i + 1 in channels.keys():
            temp_channels[i] = channels[i + 1]
    channels = temp_channels
    return dialect.MAVLink_rc_channels_override_message(vehicle.target_system, vehicle.target_component, *channels)

def safe_pwm(value, offset):
    value = value + offset
    if value < 1000:
        value = 1000
    if value > 2000:
        value = 2000
    return value


# connect to vehicle
vehicle = utility.mavlink_connection(device="tcp:127.0.0.1:5762")

# wait for a heartbeat
vehicle.wait_heartbeat()

# inform user
print("Connected to system:", vehicle.target_system, ", component:", vehicle.target_component)


# infinite loop
while True:
    channels = {1: 1500, 2: 1500, 3: 1500, 4: 1500}
    # debug the channels
    print(channels)

    # create rc channels override message
    message = override_channels(vehicle=vehicle, channels=channels)

    # send the message to the vehicle
    vehicle.mav.send(message)

    time.sleep(5)

    channels = {1: 1500, 2: 1500, 3: 1580, 4: 1500}
    # debug the channels
    print(channels)

    # create rc channels override message
    message = override_channels(vehicle=vehicle, channels=channels)

    # send the message to the vehicle
    vehicle.mav.send(message)

    time.sleep(5)