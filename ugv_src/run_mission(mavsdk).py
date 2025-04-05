import asyncio
from mavsdk import System
from mavsdk import mission_raw
from mavsdk.telemetry import Health

# Connect to Pixhawk over serial
async def connect_drone():
    print("setting drone equal to system")
    drone = System()
    print("Waiting for Serial Connection")
    await drone.connect(system_address="serial:///dev/serial0:57600")
    print("Connecting to Pixhawk...")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Pixhawk connected.")
            break

    return drone

# Wait until the system is armable and GPS is good
async def wait_for_health(drone):
    print("Waiting for vehicle to be armable (GPS, system health)...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_armable:
            print("Vehicle is healthy and ready to arm.")
            break
        await asyncio.sleep(1)

# Send a raw mission item (single waypoint)
async def send_mission(rover, lat, lon):
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
        1,                         # Waypoint index (0 = home)
        3,                       # MAV_FRAME_GLOBAL_RELATIVE_ALT
        16,                    # MAV_CMD_NAV_WAYPOINT
        0,                     # Not current item
        1,                # Continue automatically
        0,                      # Hold time (0 for rover)
        10,                     # Acceptance radius in meters
        0,                      # Pass through (0 = normal)
        float('nan'),           # Desired yaw angle (NaN = no change)
        int(lat * 1e7),              # Latitude (scaled)
        int(lon * 1e7),              # Longitude (scaled)
        30.0,                        # Altitude in meters (relative)
        0                # MAV_MISSION_TYPE_MISSION
    ))

    await asyncio.sleep(5)

    print("-- Uploading mission (raw)...")
    await rover.mission_raw.upload_mission(mission_items)
    print("-- Mission upload complete.")

# Arm the rover
async def arm_vehicle(drone):
    print("Arming rover...")
    await drone.action.arm()
    print("Rover armed.")

# Start mission (AUTO mode)
async def start_mission(drone):
    print("Starting mission (AUTO mode)...")
    await drone.mission_raw.start_mission()
    print("Mission started.")

# Monitor progress
async def monitor_mission(drone):
    print("Monitoring mission progress...")
    async for position in drone.telemetry.position():
        print(f"Current location: lat={position.latitude_deg:.6f}, lon={position.longitude_deg:.6f}")
        await asyncio.sleep(1)

# Main
async def main():
    print("Trying to Connect to Pixhawk")
    drone = await connect_drone()
    await wait_for_health(drone)

    # Replace with your desired waypoint
    waypoint_lat = 34.7794089
    waypoint_lon = -86.5670675

    await send_mission(drone, waypoint_lat, waypoint_lon)
    await asyncio.sleep(15)
    await arm_vehicle(drone)
    await asyncio.sleep(10)
    await start_mission(drone)
    await monitor_mission(drone)

if __name__ == "__main__":
    print("Running Main")
    asyncio.run(main())
