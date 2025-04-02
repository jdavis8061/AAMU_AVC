import socket
import os
import cv2 as cv
from cv2 import aruco
import numpy as np
import asyncio
#import logging
import pymavlink
#import dronekit
from mavsdk import System
from mavsdk import mission_raw
import serial
import time
import struct
import subprocess
from pymavlink import mavutil

'''Initialize Functions'''
def wait_for_wifi(timeout=60):
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

async def initialize_camera():
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
                    cap.release()
                    cv.destroyAllWindows()
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


async def initialize_pixhawk(drone):
    print("Hello Pixhawk")
    await drone.connect(system_address="serial:///dev/serial0:57600")
    
    print("Waiting for UGV drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to UGV rover!")
            break

def initialize_wifi_ff():
    print("Hello Wifi")
    global aruco_lat
    aruco_lat = 5.0
    global aruco_long
    aruco_long = 5.0
    print ("ArUco Lat & Long set to ", aruco_lat, " & ", aruco_long)
    print ("Waiting for new values from UAV...")

    HOST = '10.235.254.239'  # Listen on all available interfaces 
    PORT = 65432        # Port to listen on (non-privileged ports are > 1023) 
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:     
        s.bind((HOST, PORT))     
        s.listen()     
        conn, addr = s.accept()
        print(f"Connected by {addr}")
        data = conn.recv(8)
        aruco_lat, aruco_long = struct.unpack('ff', data)
        print(f"Received ArUco Lat={aruco_lat}, ArUco Long={aruco_long}")
        conn.close()
        s.close()
    
        '''     
        with conn:         
            print('Connected by', addr)         
            while True:             
                data = conn.recv(1024)             
                if not data:                 
                    break             
                print(data)
                conn.sendall(data)
        '''
# host = ""
# port = 5000

# s = socket.socket()
# s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) #avoid reuse error msg
# s.bind((host,port))

# print ("Server started. Waiting for connection...")
# s.listen()
# c, addr = s.accept()
# print ("Connection from: ",addr)


# while True:
#     #data is in bytes format, use decode() to transform it into a string
#     data = c.recv(1024)
#     if not data:
#         break
#     value = data.decode()
#     print ("Received: ",value)
# print ("Disconnected. Exiting.")

def initialize_wifi_dd():
    print("Hello Wifi")
    global aruco_lat
    aruco_lat = 5.0
    global aruco_long
    aruco_long = 5.0
    print ("ArUco Lat & Long set to ", aruco_lat, " & ", aruco_long)
    print ("Waiting for new values from UAV...")

    HOST = '10.235.254.239'  # Listen on all available interfaces 
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

async def initialize_mp_params(drone):
    print("Hello Mission Planner")

    all_params = any

    # Get the list of parameters
    all_params = await drone.param.get_all_params()

    # Iterate through all int parameters
    for param in all_params.int_params:
        print(f"{param.name}: {param.value}")

    # Iterate through all float parameters
    for param in all_params.float_params:
        print(f"{param.name}: {param.value}")

def initialize_logger():
    print("Hello logger")

'''Run Functions'''

'''
def run_camera():
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

'''Finalize Functions'''

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

def push_then_retract():
    push()
    retract()

'''Mission Planner Functions'''

def initialize_mp_params():
    print("Hello Mission Planner")

async def send_mission(rover, lat, long):
    mission_items = []

    mission_items.append(mission_raw.MissionItem(
        #sets 1st waypoint
        1,
        3,
        16,
        0,
        1,
        0,
        10,
        0,
        float('nan'),
        int(lat * 10**7),
        int(long * 10**7),
        30.0,
        0
    ))

    print("-- Uploading mission")
    await rover.mission_raw.upload_mission(mission_items)
    print("-- Done")

def prearms_satisfied():
    #populate later
    print("Prearm conditions satisified")

def arm_vehicle(master):
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


# Function to set mode
def set_mode(master, mode):
    # Wait for a heartbeat before sending commands
    master.wait_heartbeat()
    print("Heartbeat received from vehicle for mode selection.")

    # Get mode ID from the mode string
    mode_id = master.mode_mapping()[mode]
    
    # Send command to change mode
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    print(f"Mode set to {mode}")

    #Wait for a moment to ensure mode change
    time.sleep(2)
    print("Vehicle now in Auto mode.")

async def create_and_run_mission(rover, master, lat, long):
    # Wait for a heartbeat before sending commands
    master.wait_heartbeat()
    print("Heartbeat received from vehicle.")
    #sends mission
    await send_mission(rover, lat, long)
    #checks Prearm Little
    await prearms_satisfied()
    #arms vehicle
    arm_vehicle(master)
    # Set mode to AUTO
    await set_mode(master, "AUTO")

async def test_waypoint_creation(lat, long):
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
         int(47.40271757 * 10**7),
         # param6 - longitude
         int(8.54285027 * 10**7),
         # param7 - altitude
         30.0,
         # mission_type.
         0
     ))

    mission_items.append(mission_raw.MissionItem(
        #sets waypoint number, 0 sets home, 1 is 1st waypoint, 2 is 2nd waypoint, etc.
        1,
        # MAV_FRAME command. 3 is WGS84 + relative altitude
        3,
        # command. 16 is a basic waypoint
        16,
        # first one is current
        0,
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
        int(lat * 10**7),
        # param6 - longitude
        int(long * 10**7),
        # param7 - altitude
        30.0,
        # mission_type.
        0
    ))

    print("-- Uploading mission")
    #await drone.mission_raw.upload_mission(mission_items)
    print("mission items: ", mission_items)
    print("-- Done")

async def initialize(drone):
    #initialize_logger()
    initialize_wifi_dd()
    initialize_mp_params(drone)
    #initialize_pixhawk(drone)
    #initialize_camera()
    time.sleep(10)

async def run():
    push_then_retract()
    wait_for_wifi()
    initialize_wifi_dd()
    await create_and_run_mission()
    await initialize_camera()

async def finalize():
    print("finalize")

#asyncio.run(run())
#initialize_wifi()
#initialize_wifi_dd()

'''
if __name__ == "__main__":

    print("Starting UGV Application...")
    
    drone = System()

    asyncio.run(initialize(drone))
    asyncio.run(run())
    asyncio.run(finalize())
'''