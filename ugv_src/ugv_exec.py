import socket
import os
import cv2 as cv
from cv2 import aruco
import numpy as np
import asyncio
#import logging
#import pymavlink
#import dronekit
from mavsdk import System
import struct
import time
import subprocess
import serial
#from pymavlink import mavutil
from mavsdk import mission_raw
from mavsdk import action
from datetime import datetime
import sys
import math

#When running from boot or without a monitor, this should be the first function that runs
#There is no Pi to Pi communication or remote desktop without wifi
#When the wifi connects, most other things that cause errors due to set up time should be settled
def wait_for_wifi(timeout=60):
    "Waiting for Wifi"
    start_time = time.time()
    while time.time() - start_time < timeout:
        try:
            output = subprocess.check_output("hostname -I", shell=True).decode().strip()
            if output:
                print(f"Wifi connected! IP: {output}")
                return True
        except Exception:
            pass
        time.sleep(5)
    print("Wifi not detected after timeout. Continuing without Wifi.")
    return False

'''Camera Functions'''

def initialize_camera():
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
    push_complete = False

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
                if not push_complete and ids[0] == 23: #change based on marker id
                    print("correct marker detected")
                    push_then_retract()
                    push_complete = True
                    break
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

'''def run_camera():
    print("running camera")
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

def finalize_camera():
    print("closing camera")
    cap.release()
    cv2.destroyAllWindows()
'''

async def initialize_pixhawk(rover):
    print("Waiting for Serial Connection")
    await rover.connect(system_address="serial:///dev/serial0:57600")
    print("Serial Connection Confirmed. Connecting to Pixhawk...")

    async for state in rover.core.connection_state():
        if state.is_connected:
            print("Pixhawk connected.")
            break

    return rover

def initialize_wifi():
    print("Hello Wifi")
    global aruco_lat
    aruco_lat = 5.0
    global aruco_long
    aruco_long = 5.0
    print ("ArUco Lat & Long temporarily set to ", aruco_lat, " & ", aruco_long)
    print ("Waiting for new values from UAV...")

    HOST = '192.168.134.221'  # Listen on all available interfaces 
    PORT = 65432        # Port to listen on (non-privileged ports are > 1023) 
    server_s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)     
    server_s.bind((HOST, PORT))     
    server_s.listen()     
    conn, addr = server_s.accept()
    print(f"Connected by {addr}")
    data = conn.recv(16)
    aruco_lat, aruco_long = struct.unpack('dd', data)
    print(f"Received ArUco Lat={aruco_lat}, ArUco Long={aruco_long}")
    conn.close()
    server_s.close()

'''Mission Planner Functions'''

def initialize_mp_params():
    print("Hello Mission Planner")

async def get_current_location(rover):
    async for position in rover.telemetry.position():
        return position.latitude_deg, position.longitude_deg


# Wait until the system is armable and GPS is good
async def wait_for_health(drone):
    print("Waiting for vehicle to be armable (GPS, system health)...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_armable:
            print("Vehicle is healthy and ready to arm.")
            break
        await asyncio.sleep(1)

# Send a raw mission item (single waypoint)
async def send_mission(rover, home_lat, home_long, lat, lon, max_retries=5, retry_delay=4):
    mission_items = []

    mission_items.append(mission_raw.MissionItem(
         # start seq at 0/sets home
         0,
         # MAV_FRAME command. 3 is WGS84 + relative altitude
         3,
         # command. 16 is a basic waypoint
         16,
         # first one is current
         1,
         # auto-continue. 1: True, 0: False
         1,
         # param1
         0,
         # param2 - Acceptance radius
         10,
         # param3 - 0 (pass through the waypoint normally)
         0,
         # param4 - Desired yaw angle at waypoint
         float('nan'),
         # param5 - latitude
         int(home_lat * 10**7),
         # param6 - longitude
         int(home_long * 10**7),
         # param7 - altitude
         30.0,
         # mission_type.
         0
     ))

    mission_items.append(mission_raw.MissionItem(
        1,                         # Waypoint index (0 = home)
        3,                       # MAV_FRAME_GLOBAL_RELATIVE_ALT
        16,                    # MAV_CMD_NAV_WAYPOINT
        0,                     # Not current item
        1,                # Continue automatically
        0,                      # Hold time (0 for rover)
        10,                     # Acceptance radius in meters
        0,                      # Pass through (0 = normal)
        float('nan'),           # Desired yaw angle (NaN = no change)
        int(lat * 1e7),              # Latitude (scaled)
        int(lon * 1e7),              # Longitude (scaled)
        30.0,                        # Altitude in meters (relative)
        0                # MAV_MISSION_TYPE_MISSION
    ))

    await asyncio.sleep(5)

    # Retry mechanism
    attempt = 0
    while attempt < max_retries:
        try:
            print(f"-- Uploading mission (attempt {attempt + 1})...")
            await rover.mission_raw.upload_mission(mission_items)
            print("-- Mission upload complete.")
            break  # Success
        except Exception as e:
            print(f"? Upload failed on attempt {attempt + 1}: {e}")
            attempt += 1
            if attempt < max_retries:
                print(f"? Retrying in {retry_delay} seconds...")
                await asyncio.sleep(retry_delay)
            else:
                print("? Max retries reached. Mission upload failed.")
    '''try:
        print("-- Uploading mission (raw)...")
        await rover.mission_raw.upload_mission(mission_items)
        print("-- Mission upload complete.")
    except Exception as e:
        print(f"? Failed to upload mission: {e}")'''

# Arm the rover
async def arm_vehicle(drone):
    print("Arming rover...")
    await drone.action.arm()
    print("Rover armed.")

# Start mission (AUTO mode)
async def start_mission(drone):
    print("Starting mission (AUTO mode)...")
    await drone.mission_raw.start_mission()
    print("Mission started.")
# Stream GPS updates until mission is finished
async def stream_position_until_done(drone, mission_complete):
    async for position in drone.telemetry.position():
        print(f"Current location: lat={position.latitude_deg:.6f}, lon={position.longitude_deg:.6f}")
        await asyncio.sleep(1)
        if mission_complete.is_set():
            break

# Monitor mission progress via mission_raw
async def wait_for_mission_completion(drone, mission_complete):
    last_reported = -1
    mission_started = False

    async for progress in drone.mission_raw.mission_progress():
        if progress.current != last_reported:
            print(f"Home/Launch location =1. Reached waypoint {progress.current} of {progress.total}")
            last_reported = progress.current

        # Ignore initial home (0) and first waypoint (1)
        if progress.current >= 2 or (progress.total == 2 and progress.current == 1 and mission_started):
            print("Mission Complete.")
            mission_complete.set()
            break

        # Flag that mission has started once we move off home
        if progress.current > 0:
            mission_started = True

# Monitor mission and stop position stream when done
async def monitor_mission(drone):
    print("Monitoring mission progress...")
    mission_complete = asyncio.Event()

    # Run both tasks in parallel
    await asyncio.gather(
        stream_position_until_done(drone, mission_complete),
        wait_for_mission_completion(drone, mission_complete)
    )

def calculate_bearing_to_home(current_lat, current_lon, home_lat, home_lon):
    # Convert to radians
    lat1 = math.radians(current_lat)
    lat2 = math.radians(home_lat)
    diff_long = math.radians(home_lon - current_lon)

    x = math.sin(diff_long) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (
        math.sin(lat1) * math.cos(lat2) * math.cos(diff_long)
    )

    initial_bearing = math.atan2(x, y)
    initial_bearing = math.degrees(initial_bearing)
    compass_bearing = (initial_bearing + 360) % 360
    return compass_bearing

async def face_home(rover, home_lat, home_long):
    current_lat, current_long = await get_current_location(rover)
    heading = calculate_bearing_to_home(current_lat, current_long, home_lat, home_long)

    # Keep current lat/lon, just turn to face the heading
    await rover.action.goto_location(
        current_lat,
        current_long,
        0,             # Altitude isn't used for ground vehicles
        heading        # Yaw (degrees)
    )
    print(f"?? Turning to face home (heading {heading:.2f}�)...")
    await asyncio.sleep(5)  # Give time to complete the turn


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
    log_dir = "/home/jdavis/Rover_Logs"
    log_filename = os.path.join(log_dir, f"ugv_log_{timestamp}.log")

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

'''Actuator Functions'''

def send_command(channel, target):
    ACTUATOR_SERIAL_PORT = "/dev/serial/by-id/usb-Pololu_Corporation_Pololu_Micro_Maestro_6-Servo_Controller_00454274-if00"
    ACTUATOR_BAUD_RATE = 9600
    command = bytearray([0x84, channel, target & 0x7F, (target >> 7) & 0x7F])
    with serial.Serial(ACTUATOR_SERIAL_PORT, ACTUATOR_BAUD_RATE, timeout=1) as ser:
        ser.write(command)
        time.sleep(0.1)

def push():
    print("Activating Servo...")
    send_command(0, 8000)
    time.sleep(7)

def retract():
    print("Retracting Servo...")
    send_command(0, 4000)
    time.sleep(7)

async def push_then_retract():
    push()
    retract()
    print("Push complete")

async def initialize(rover):
    print("initializing")
    wait_for_wifi()
    await initialize_pixhawk(rover)
    await initialize_wifi()
    #initialize_camera()
    #initialize_mp_params()
    print("Intialization Complete")

async def run(rover, aruco_lat, aruco_long):
    print("Starting run function")

    global home_lat, home_long
    home_lat, home_long = await get_current_location(rover)

    # Replace with your desired waypoint
    await send_mission(rover, home_lat, home_long, aruco_lat, aruco_long)

    #print (home_lat, ' & ', home_long)
    await wait_for_health(rover)

    await asyncio.sleep(5)
    await arm_vehicle(rover)
    await asyncio.sleep(5)
    await start_mission(rover)
    await monitor_mission(rover)
    await face_home(rover, home_lat, home_long)
    await face_home(rover, home_lat, home_long)
    await face_home(rover, home_lat, home_long)
    await push_then_retract()
    print("Run function complete")



async def finalize(rover):
    print("Beginning Finalize Function")
    print("Sending Return To Launch (RTL) command...")
    await rover.action.return_to_launch()
    print("RTL command sent. Returning Home")

async def main():
    
    rover = System()

    aruco_lat = 32.7289889 #34.7802020 #temporary until test with UAV
    aruco_long = -97.1265730 #-86.5668093 #temporary until test with UAV
    await initialize(rover)
    await run(rover, aruco_lat, aruco_long)
    await finalize(rover)


if __name__ == "__main__":
    initialize_logger()

    try:
        print("Starting UGV Application...")
    
        rover = System()

        asyncio.run(main())
    
    except Exception as e:
        print(f"[{datetime.now()}] ERROR: {e}")

    finally:
        #Logs end time
        finalize_logger(log_file)