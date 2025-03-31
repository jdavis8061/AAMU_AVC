import socket
import os
import cv2 as cv
from cv2 import aruco
import numpy as np
import asyncio
import logging
from pymavlink import mavutil
#import roverkit
from mavsdk import System
import struct
import time
import subprocess
import serial
from mavsdk import mission_raw

'''Wifi, Peer to Peer Functions, and Logger'''
def initialize_logger():
    print("Hello logger")

#When running from boot or without a monitor, this should be the first function(besides logger) that runs
#There is no Pi to Pi communication or remote desktop without wifi
#When the wifi connects, most other things that cause errors due to set up time should be settled
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

#Peer to Peer Wifi Connection - Expects Real GPS Coordinates from UAV
def initialize_wifi():
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

'''Pixhawk & Mission Planner Functions'''

async def initialize_pixhawk(rover):
    print("Hello Pixhawk")
    await rover.connect(system_address="serial:///dev/serial0:57600")
    print("Waiting for rover to connect...")
    async for state in rover.core.connection_state():
        if state.is_connected:
            print("-- Connected to rover!")
            return rover


def initialize_mp_params():
    print("Hello Mission Planner")

async def send_real_waypoints(rover, lat, long):
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

async def send_fake_mission(rover):
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
        int(34.7799771 * 10**7),
        int(-86.5669265 * 10**7),
        30.0,
        0
    ))

    print("-- Uploading mission")
    await rover.mission_raw.upload_mission(mission_items)
    print("-- Done")

async def arm_vehicle(master):
    """Connect via pymavlink and arm the vehicle."""
    
    print("Arming vehicle...")

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )

    time.sleep(2)  # Wait for the vehicle to arm
    print("Vehicle armed successfully.")


async def set_auto_mode(master):
    """Set vehicle mode to AUTO using pymavlink."""

    print("-- Setting vehicle to AUTO mode...")

    mode_mapping = master.mode_mapping()
    if "AUTO" not in mode_mapping:
        print(f"Mode AUTO not found! Available modes: {mode_mapping.keys()}")
        return

    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_mapping["AUTO"]
    )

    time.sleep(2)  # Wait for mode change
    print("Vehicle is now in AUTO mode.")


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

'''Initialize, Run, and Finalize Functions'''
async def initialize(rover):
    print("initializing")
    #await initialize_logger()        #starts logging data
    await wait_for_wifi()             #waits for wifi to connect before running anything else
    '''We want the server to run, and the pixhawk to connect in the background. Test .gather, .create_task'''
    '''.gather test'''
    #await asyncio.gather(initialize_wifi(), rover = initialize_pixhawk(rover), return_exceptions=True) #runs initialize wifi and initial pixhawk at the same time. Waits for both to finish before continuing
    '''.create_tasks test'''
    #await GPS_received = asyncio.create_task(initialize_wifi)
    #await rover = await initialize_pixhawk(rover)
    '''if none work'''
    #await initialize_wifi()           #starts the server to listen
    #rover = await initialize_pixhawk(rover)       #while server is listening connect to the pixhawk
    #initialize_mp_params()     #recieve all current mission planner parameters

async def run(rover, master):
    print("running")
    await send_real_waypoints(rover, aruco_lat, aruco_long) #sends the waypoint to the pixhawk
    #await prearm_checks_cleared() #checks prearms before attempting to arm
    await arm_vehicle(master) #arms vehicle before attempting to set the mode to auto
    await set_auto_mode(master) #sets vehicle to auto mode before attempting anything else
    #await received_mission_complete() #waits for mission complete message before finishing the mission
    

async def finalize():
    print("finalizing")
    await push_then_retract() #waits for servo
    #await return_home()


print("Hello UGV World")

if __name__ == "__main__":

    print("Starting UGV Application...")
    
    rover = System()

    asyncio.run(initialize(rover))

    print("Connecting via pymavlink for Mission Planning.")
    master = mavutil.mavlink_connection('/dev/serial0', baud=57600)
    master.wait_heartbeat()
    print("Heartbeat received.")

    asyncio.run(run(rover, master))
    asyncio.run(finalize())
