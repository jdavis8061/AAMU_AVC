#!/usr/bin/env python3

import socket
import asyncio
from mavsdk import System
#from mavsdk.action import FlightMode
import sys
import struct
from pymavlink import mavutil
import time
from datetime import datetime
import cv2 as cv
from cv2 import aruco
import numpy as np


# import logging'''
import os
# '''import pymavlink
# #import dronekit

HOST = '192.168.1.6'    # The server's hostname or IP address 
PORT = 65432              # The same port as used by the server 
TARGET_MARKER_ID = 4
MARKER_SIZE = 8  # in cm

async def old_initialize_camera(drone):
    calib_data_path = os.path.join(os.path.dirname(__file__), "..", "MultiMatrix.npz")

    calib_data = np.load(calib_data_path)
    print(calib_data.files)
    cam_mat = calib_data["camMatrix"]
    dist_coef = calib_data["distCoef"]
    r_vectors = calib_data["rVector"]
    t_vectors = calib_data["tVector"]


    MARKER_SIZE = 8  

    marker_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)

    param_markers = aruco.DetectorParameters()

    #cap = cv.VideoCapture("http://10.235.100.3:8080/video") #give the server id shown in IP webcam App
    cap = cv.VideoCapture(0) #uses USB Camera Added (Change to 0 when using Raspberry Pi, 1 when using laptop)

    if not cap.isOpened():
        print("❌ Failed to open camera")
        exit()
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        marker_corners, marker_IDs, reject = aruco.detectMarkers(
            gray_frame, marker_dict, parameters=param_markers
        )
        if marker_corners:
            rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
                marker_corners, MARKER_SIZE, cam_mat, dist_coef
            )
            total_markers = range(0, marker_IDs.size)
            for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
                if ids[0] == 23:
                    print("correct marker detected")
                    cap.release()
                    cv.destroyAllWindows()
                    return
                    #get_gps_mavsdk(drone)
                    #await initialize_wifi_with_gps_numbers(drone)
                    #break

                cv.polylines(
                    frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
                )
                corners = corners.reshape(4, 2)
                corners = corners.astype(int)
                top_right = corners[0].ravel()
                top_left = corners[1].ravel()
                bottom_right = corners[2].ravel()
                bottom_left = corners[3].ravel()


            
                # calculate the distance
                distance = np.sqrt(
                    tVec[i][0][2] ** 2 + tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2
                )



                # for pose of the marker
                point = cv.drawFrameAxes(frame, cam_mat, dist_coef, rVec[i], tVec[i], 4, 4)
                cv.putText(
                    frame,
                    f"id: {ids[0]} Dist: {round(distance, 2)}",
                    top_right,
                    cv.FONT_HERSHEY_PLAIN,
                    1.3,
                    (0, 0, 255),
                    2,
                    cv.LINE_AA,
                
                )
                cv.putText(
                    frame,
                    f"x:{round(tVec[i][0][0],1)} y: {round(tVec[i][0][1],1)} ",
                    bottom_right,
                    cv.FONT_HERSHEY_PLAIN,
                    1.0,
                    (0, 0, 255),
                    2,
                    cv.LINE_AA,
                )
                # print(ids, "  ", corners)
        cv.imshow("frame", frame)
        key = cv.waitKey(1)
        if key == ord("q"):
            break
    cap.release()
    cv.destroyAllWindows()

def initialize_camera():
    calib_data_path = os.path.join(os.path.dirname(__file__), "..", "MultiMatrix.npz")
    calib_data = np.load(calib_data_path)

    cam_mat = calib_data["camMatrix"]
    dist_coef = calib_data["distCoef"]

    marker_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
    param_markers = aruco.DetectorParameters()

    cap = cv.VideoCapture(0)  # Change to 1 if using laptop cam
    if not cap.isOpened():
        print("? Failed to open camera")
        return None, None, None, None

    return cap, cam_mat, dist_coef, (marker_dict, param_markers)

def detect_aruco_marker(cap, cam_mat, dist_coef, aruco_settings):
    marker_dict, param_markers = aruco_settings

    ret, frame = cap.read()
    if not ret:
        return False

    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    marker_corners, marker_IDs, _ = aruco.detectMarkers(
        gray_frame, marker_dict, parameters=param_markers
    )

    if marker_IDs is not None:
        for ids in marker_IDs:
            print("Reading Marker: ", ids[0])
            if ids[0] == TARGET_MARKER_ID:
                print(f"Correct Marker ID {TARGET_MARKER_ID} detected!")
                return True


    # Display the frame (optional)
    cv.imshow("Aruco Detection", frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
        return False  # Treat 'q' as exit

    return False

async def run_camera():
    print("?? Scanning for ArUco marker...")

    # Continuously scan for marker
    while True:
        if detect_aruco_marker(cap, cam_mat, dist_coef, aruco_settings):
            break
        await asyncio.sleep(0.1)  # Let asyncio breathe

async def initialize_pixhawk(drone):
    print("trying to connect to pixhawk via serial")
    await drone.connect(system_address="serial:///dev/ttyAMA0:57600")
    
    print("Waiting for UAV drone to connect...")
    async for state in drone.core.connection_state():
        print("trying to connect")
        if state.is_connected:
            print(f"-- Connected to UAV drone!")
            break

# Wait until the system is armable and GPS is good
async def wait_for_health(drone):
    print("Waiting for vehicle to be armable (GPS, system health)...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_armable:
            print("Vehicle is healthy and ready to arm.")
            break
        #await asyncio.sleep(1)

def get_gps():
    master=mavutil.mavlink_connection('/dev/serial0', baud=57600)
    master.wait_heartbeat()
    print("Connected to Vehicle")
    while True:
        print("looking for message")
        msg = master.recv_match(type='GPS_RAW_INT', blocking=True)
        print("message received")
        print(msg)
        if msg:
            global gps_latitude, gps_longitude
            gps_latitude = msg.lat / 1e7
            gps_longitude = msg.lon / 1e7
            print(f'gps Lat:{gps_latitude}, gps long:{gps_longitude}')
            return gps_latitude, gps_longitude

async def get_gps_mavsdk(drone):
    print("Fetching GPS coordinates...")
    global gps_latitude, gps_longitude
    gps_latitude = 0.0
    gps_longitude = 0.0
    print("global variables created, starting telemetry reading")
    async for position in drone.telemetry.position():
            print("pulling gps")
            gps_latitude = position.latitude_deg
            gps_longitude = position.longitude_deg
            print(f"Lat: {gps_latitude}, Long: {gps_longitude}")
            #return gps_latitude, gps_longitude
            break

""" def initialize_wifi():
    print("Hello Wifi")

    # s = socket.socket() 
    with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as s:     
        s.connect((HOST, PORT))
        print("Wifi connected to UGV at ",HOST, ":", PORT) 
        # s.sendall(b'Hello, world')     
        # data = s.recv(1024) 
        #     s.send(message.encode()) #convert to bytes then send 
        # print('Sent', repr(data))

def initialize_wifi_with_ex_numbers(lat, long):
    print("Hello Wifi")

    # s = socket.socket() 
    with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as s:     
        s.connect((HOST, PORT))
        print("Wifi connected to UGV at ",HOST, ":", PORT) 
        # s.sendall(b'Hello, world')     
        # data = s.recv(1024) 
        #     s.send(message.encode()) #convert to bytes then send 
        # print('Sent', repr(data))
        aruco_coordinates = struct.pack('ff', lat, long)
        s.send(aruco_coordinates)
        s.close
 """
async def initialize_wifi_with_gps_numbers(drone):
    print("Attempting to connect to UGV")
    # s = socket.socket() 
    client_s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)    
    client_s.connect((HOST, PORT))
    print("Wifi connected to UGV at ",HOST, ":", PORT) 
        # s.sendall(b'Hello, world')     
        # data = s.recv(1024)
        #     s.send(message.encode()) #convert to bytes then send 
        # print('Sent', repr(data))
    """ gps_latitude = 32.1234567
    gps_longitude = 97.1234567 """
    gps_coordinates = struct.pack('dd', gps_latitude, gps_longitude)
    client_s.send(gps_coordinates)
    print("latitude & longitude sent")
    client_s.close()

async def initialize_mp_params(drone):
    print("Hello Mission Planner")

    all_params = any
    print("all_params variable set")
    # Get the list of parameters
    all_params = await drone.param.get_all_params()
    print("params recieved")

    # Iterate through all int parameters
    for param in all_params.int_params:
        print(f"{param.name}: {param.value}")

    # Iterate through all float parameters
    for param in all_params.float_params:
        print(f"{param.name}: {param.value}")

async def force_loiter(drone):
    LOITER_DURATION = 10  # seconds
    # Marker detected: switch to LOITER
    print("?? Switching to LOITER mode...")
    await drone.action.set_flight_mode("LOITER")

""" async def force_loiter_mavsdk(drone):
    LOITER_DURATION = 10  # seconds
    # Marker detected: switch to LOITER
    print("?? Switching to LOITER mode...")
    await drone.action.set_flight_mode(FlightMode.LOITER) """


    #print(f"? Loitering for {LOITER_DURATION} seconds...")
    #await asyncio.sleep(LOITER_DURATION)

'''Logging Functions'''
class Tee:
    def __init__(self, *streams):
        self.streams = streams
    def write(self, data):
        for stream in self.streams:
            stream.write(data)
            stream.flush()
    def flush(self):
        for stream in self.streams:
            stream.flush()

def initialize_logger():
    print("Creating Log File")
    # Create a timestamped log filename in /home/jdavis
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    log_dir = "/home/deitrich1/Drone_Logs"
    os.makedirs(log_dir, exist_ok=True)
    log_filename = os.path.join(log_dir, f"uav_log_{timestamp}.log")

    # Open the log file and redirect stdout and stderr
    global log_file
    log_file = open(log_filename, "w")
    sys.stdout = Tee(sys.__stdout__, log_file)
    sys.stderr = Tee(sys.__stderr__, log_file)
    print("Log File Created")

    # Log the start time   
    print(f"[{datetime.now()}] --- Script STARTED ---")


def finalize_logger(log_file):
    print(f"[{datetime.now()}] --- Script ENDED --- ")
    log_file.close()


async def initialize(drone):
    print("running initialize functions")
    await initialize_pixhawk(drone)
    global cap, cam_mat, dist_coef, aruco_settings
    cap, cam_mat, dist_coef, aruco_settings = initialize_camera()
    if cap is None:
        print("? Camera initialization failed.")
        return
    print("camera initialized")
    #await initialize_wifi(drone)
    #await initialize_mp_params(drone)
    #print("params printed")
    """await get_gps_mavsdk(drone)
    await old_initialize_camera(drone)
    await get_gps_mavsdk(drone) """
    print("finished initializing")

async def run(drone):
    print("executing run function")
    #await wait_for_health(drone)
    await run_camera()
    #await force_loiter(drone)
    await get_gps_mavsdk(drone)
    await initialize_wifi_with_gps_numbers(drone)
    print("run function complete")

async def finalize(drone):
    print("executing finalize function")

    #print("sending land command")
    #await drone.action.land()
    
    print("closing camera")
    cap.release()
    cv.destroyAllWindows()
    print("camera closed")


    # Cancel the mission by clearing it
    """ print("?? Clearing remaining mission items...")
    await drone.mission.clear_mission()
    print("mission items cleared") """

    # Return to launch
    """ print("Commanding Return to Launch (RTL)...")    
    await drone.action.return_to_launch()
    print("return to launch set, waiting for drone to land") """
    print("waiting for landing")
    async for in_air in drone.telemetry.observe_in_air():
        if not in_air:
            print("Drone landed!")
            break

    print("finalize function complete")

#initialize_wifi_with_gps_numbers()
#time.sleep(10)

async def main():
    drone = System()
    
    await initialize(drone)
    await run(drone)
    await finalize(drone)

if __name__ == "__main__":
    initialize_logger()

    try:
        print("Starting UAV Application...")

        asyncio.run(main())
    
    except Exception as e:
        print(f"ERROR: {e}")

    finally:
        #Logs end time
        finalize_logger(log_file)
        print("done")

""" if __name__ == "__main__":

    print("Starting UAV Application...")
    
    drone = System()
    print("drone = system")
    asyncio.run(initialize(drone))
    #asyncio.run(run())
    #asyncio.run(finalize())

    #get_gps() """