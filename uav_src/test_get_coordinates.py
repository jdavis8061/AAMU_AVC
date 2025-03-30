from pymavlink import mavutil

#must be connected to Pixhawk, GPS, and Power Supply to run this code
master=mavutil.mavlink_connection('/dev/serial0', baud=57600)

master.wait_heartbeat()
print("Connected to Vehicle")
def get_gps():
    while True:
        msg = master.recv_match(type='GPS_RAW_INT', blocking=True)
        if msg:
            aruco_latitude = msg.lat / 1e7
            aruco_longitude = msg.lon / 1e7
            print(f'ArUco Lat:{aruco_latitude}, aruco longitude:{aruco_longitude}')
            return aruco_latitude, aruco_longitude

aruco_latitude, aruco_longitude = get_gps()