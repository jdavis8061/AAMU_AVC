from pymavlink import mavutil

#must be connected to Pixhawk, GPS, and Power Supply to run this code 
def get_gps():
    master=mavutil.mavlink_connection('/dev/serial0', baud=57600)
    master.wait_heartbeat()
    print("Connected to Vehicle")
    while True:
        msg = master.recv_match(type='GPS_RAW_INT', blocking=True)
        if msg:
            gps_latitude = msg.lat / 1e7
            gps_longitude = msg.lon / 1e7
            print(f'ArUco Lat:{gps_latitude}, aruco longitude:{gps_longitude}')
            return gps_latitude, gps_longitude

drone_latitude, drone_longitude = get_gps()