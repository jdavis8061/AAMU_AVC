from pymavlink import mavutil
import time

# Connect to the Pixhawk
master = mavutil.mavlink_connection('/dev/serial0', baud=57600)

# Wait for the heartbeat message from the Pixhawk
master.wait_heartbeat()

print("Heartbeat received from vehicle. Arming...")

# Send the arm command
master.mav.command_long_send(
    master.target_system,    # target_system
    master.target_component, # target_component
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, # command to arm/disarm
    0,                       # confirmation
    1,                       # arm the vehicle (0 for disarm)
    0, 0, 0, 0, 0, 0         # unused parameters
)

# Wait a bit to ensure the vehicle arms
time.sleep(2)

print("Vehicle armed successfully.")
