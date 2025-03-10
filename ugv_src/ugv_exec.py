
import cv2 as cv
from cv2 import aruco
import numpy as np
import asyncio
import logging
import os
import pymavlink
#import dronekit
import test_pusher
from mavsdk import System
    
def initialize_camera():
    calib_data_path = r"C:\Users\allst\OneDrive\Desktop\AAMU_AVC\AAMU_AVC\ugv_src\MultiMatrix.npz"

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
    cap = cv.VideoCapture(1) #uses USB Camera Added (Change to 0 when using Raspberry Pi, 1 when using laptop)
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
                if not push_complete and ids[0] == 23:
                    print("correct marker detected")
                    test_pusher.activate_servo()
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
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("❌ Failed to open camera")
        exit()
#move to run later
    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ Failed to grab frame")
            break

        cv2.imshow("Camera Feed", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
#move to finalize later
    cap.release()
    cv2.destroyAllWindows()
    '''

def run_camera():
    print("running camera")
    '''
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
        '''

def finalize_camera():
    print("closing camera")
    '''
    cap.release()
    cv2.destroyAllWindows()
    '''

async def initialize_pixhawk():
    print("Hello Pixhawk")
    drone = System()
    await drone.connect()

def initialize_wifi():
    print("Hello Wifi")

def initialize_mp_params():
    print("Hello Mission Planner")

def initialize_logger():
    print("Hello logger")

def initialize():
    print("initialize")
    #initialize_logger()
    #initialize_pixhawk()
    #initialize_wifi()
    initialize_camera()
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
