#!/usr/bin/env python3

import socket
import asyncio
from mavsdk import System
import sys
import struct

import cv2 as cv
from cv2 import aruco
import numpy as np

# import logging'''
import os
# '''import pymavlink
# #import dronekit

HOST = '10.235.254.239'    # The server's hostname or IP address 
PORT = 65432              # The same port as used by the server 


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

async def initialize_pixhawk(drone):

    await drone.connect(system_address="serial:///dev/ttyACM0:57600")
    
    print("Waiting for UAV drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to UAV drone!")
            break


def initialize_wifi():
    print("Hello Wifi")

    # s = socket.socket() 
    with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as s:     
        s.connect((HOST, PORT))
        print("Wifi connected to UGV at ",HOST, ":", PORT) 
        # s.sendall(b'Hello, world')     
        # data = s.recv(1024) 
        #     s.send(message.encode()) #convert to bytes then send 
        # print('Sent', repr(data))

def initialize_wifi_with_numbers(lat, long):
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

async def initialize_logger():
     print("Hello logger")


async def initialize(drone):

    
    # initialize_logger()
    await initialize_pixhawk(drone)
    #initialize_wifi()
    await initialize_mp_params(drone)
    await initialize_camera()

async def run():
     print("run")

async def finalize():
     print("finalize")

#initialize_wifi_with_numbers(30, 86)

if __name__ == "__main__":

    print("Starting UAV Application...")
    
    drone = System()

    asyncio.run(initialize(drone))
    asyncio.run(run())
    asyncio.run(finalize())