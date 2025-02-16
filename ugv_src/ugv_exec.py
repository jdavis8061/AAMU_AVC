#import asyncio
#import logging
#from mavsdk import System
#from ArducamDepthCamera import ArducamCamera, Connection

def initialize_camera():
    print("Hello camera")
    #Creates an instance of the camera
    #camera = ArducamCamera()
    #ret = camera.open(Connection.MODE_USB)
    #if ret == 0:
    #    print("Camera initialized successfully.")
    #else:
    #    print(f"Failed to initialize camera. Error code: {ret}")


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

def initialize():
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