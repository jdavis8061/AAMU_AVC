import socket
'''import cv2 as cv
from cv2 import aruco
import numpy as np
import asyncio
import logging'''
import os
'''import pymavlink
#import dronekit
from mavsdk import System

print("Hello UAV World")
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

def initialize_pixhawk():
    print("Hello Pixhawk")
#    drone = System()
#    await drone.connect()
'''
def initialize_wifi():
    print("Hello Wifi")
    # host = '10.235.153.196' 
    # port = 5000 
    # s = socket.socket() 
    # s.connect((host,port)) 
    # print("Connected to",host) 
    # while True: 
    #     message = input("->") 
    #     s.send(message.encode()) #convert to bytes then send

    HOST = '10.235.194.195'    # The server's hostname or IP address 
    PORT = 65432              # The same port as used by the server 
    with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as s:     
        s.connect((HOST, PORT))     
        s.sendall(b'Hello, world')     
        data = s.recv(1024) 
        
        print('Sent', repr(data))


# Change this IP to match your drone's IP address (default is usually 192.168.4.1)
'''DRONE_IP = "192.168.4.1"

async def run():
    drone = System()
    print(f"Connecting to drone at {DRONE_IP}...")
    
    await drone.connect(system_address=f"udp://{DRONE_IP}:14550")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✅ Successfully connected to the drone!")
            break
    
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("✅ Drone is ready for flight")
            break

    async for position in drone.telemetry.position():
        print(f"📍 Drone Position: {position.latitude_deg}, {position.longitude_deg}")
        break

    await drone.action.arm()
    print("🛠 Drone Armed!")

    await drone.action.takeoff()
    print("🚀 Takeoff!")

    await asyncio.sleep(5)  # Keep script running for observation

    await drone.action.land()
    print("🛬 Landing...")

# Run the script
asyncio.run(run())

def initialize_mp_params():
    print("Hello Mission Planner")

def initialize_logger():
    print("Hello logger")

'''    
def initialize():
    print("initialize")
    #initialize_logger()
    #initialize_pixhawk()
    initialize_wifi()
    #initialize_camera()
    #initialize_mp_params()
'''
def run():
    print("run")

def finalize():
    print("finalize")
'''
initialize()