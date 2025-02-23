import asyncio
from mavsdk import System

print("Hello UAV World")
def initialize_camera():
    print("Hello camera")

def initialize_pixhawk():
    print("Hello Pixhawk")
#    drone = System()
#    await drone.connect()

def initialize_wifi():
    print("Hello Wifi")


# Change this IP to match your drone's IP address (default is usually 192.168.4.1)
DRONE_IP = "192.168.4.1"

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