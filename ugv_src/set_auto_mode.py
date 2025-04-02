from pymavlink import mavutil
import time

# Connect to the Pixhawk
master = mavutil.mavlink_connection('/dev/serial0', baud=57600)

# Wait for a heartbeat before sending commands
master.wait_heartbeat()
print("Heartbeat received from vehicle.")

# Function to set mode
def set_mode(mode):
    # Get mode ID from the mode string
    mode_id = master.mode_mapping()[mode]
    
    # Send command to change mode
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    print(f"Mode set to {mode}")

# Set mode to AUTO
set_mode("AUTO")

# Wait for a moment to ensure mode change
time.sleep(2)
print("Vehicle is now in AUTO mode.")
