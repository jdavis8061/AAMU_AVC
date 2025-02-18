
import cv2
#import asyncio
#import logging
#from mavsdk import System
#import ArducamDepthCamera
#print(dir(ArducamDepthCamera.Connection))
#from ArducamDepthCamera import ArducamCamera, Connection

def initialize_camera_DC():
    print("Hello camera")
    #Creates an instance of the camera
    
    camera = ArducamCamera()
    ret = camera.open(Connection.USB)
    if ret == 0:
        print("Camera initialized successfully.")
    else:
        print(f"Failed to initialize camera. Error code: {ret}")
    
def initialize_camera_cv2():
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

def initialize_pixhawk():
    print("Hello Pixhawk")
#    drone = System()
#    await drone.connect()

def initialize_wifi():
    print("Hello Wifi")

def initialize_mp_params():
    print("Hello Mission Planner")

def initialize_logger():
    print("Hello logger")

"""def initialize():
    print("initialize")
    initialize_logger()
    initialize_pixhawk()
    initialize_wifi()
    initialize_camera()
    initialize_mp_params()

def run():
    print("run")

def finalize():
    print("finalize")


print("Hello UGV World")
initialize()
run()
finalize()
"""
initialize_camera_cv2()