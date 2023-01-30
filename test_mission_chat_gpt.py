import os
import time
import pymavlink.mavutil as utility

# Connection information
device = "udp:127.0.0.1:14551"
baudrate = 921600

# Waypoints file
waypoints_file = "waypoints.txt"

# Create the connection object
connection = utility.mavlink_connection(device, baudrate=baudrate)

# Wait for a heartbeat to confirm the connection
while True:
    msg = connection.recv_match(blocking=True)
    if msg is not None:
        if msg.get_type() == "HEARTBEAT":
            print("Vehicle connected")
            break

# Arm the vehicle
connection.arducopter_arm()

# Read waypoints from file
with open(waypoints_file) as f:
    waypoints = f.readlines()
waypoints = [w.strip().split(',') for w in waypoints]

# Upload the waypoints to the vehicle
for wp in waypoints:
    wp_cmd = utility.mavlink.MAVLink_mission_item_message(
        0, 0, 0, 0, 0, 0, 0,
        float(wp[0]), float(wp[1]), float(wp[2]),
        0, 0, 0, 0, 0, 0
    )
    connection.mav.send(wp_cmd)

# Start mission
connection.mav.mission_start_send(connection.target_system, connection.target_component)
