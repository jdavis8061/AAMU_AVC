print("Hello UAV World")
def initialize_camera():
    print("Hello camera")

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

initialize()