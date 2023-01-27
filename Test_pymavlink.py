from pymavlink import mavutil

the_connection = mavutil.mavlink_connection("udpin:127.0.0.1:14551")

the_connection.wait_heartbeat()
print("heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))