import socket
import os
import cv2 as cv
from cv2 import aruco
import numpy as np
import asyncio
import logging
import pymavlink
#import dronekit
import actuator
from mavsdk import System
import struct
import time
import subprocess

#When running from boot or without a monitor, this should be the first function that runs
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
                    actuator.push_then_retract()
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

async def initialize_pixhawk():
    print("Hello Pixhawk")
    drone = System()
    await drone.connect()

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


def initialize_mp_params():
    print("Hello Mission Planner")

def initialize_logger():
    print("Hello logger")

def initialize():
    print("initialize")
    #initialize_logger()
    #initialize_pixhawk()
    #initialize_wifi()
    #initialize_camera()
    #initialize_mp_params()

def run():
    print("run")

def finalize():
    print("finalize")


print("Hello UGV World")
print(os.environ)
initialize()
#run()
#finalize()
