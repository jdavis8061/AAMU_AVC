from mavsdk.mission import MissionItem

def Create_MissionItem(lat, lng):
    mission_items = [
            MissionItem(
                latitude_deg=lat / 1e7,  # Latitude
                longitude_deg=lng / 1e7,  # Longitude
                relative_altitude_m=2.0,  # Relative altitude (meters)
                speed_m_s=2.0,  # Speed (m/s)
                is_fly_through=False,  # Stop at waypoint
                gimbal_pitch_deg=0.0,
                gimbal_yaw_deg=0.0,
                loiter_time_s=0.0,  # Loiter time at waypoint (seconds)
                camera_photo_interval_s=0.0,  # Optional, set to 0.0
                acceptance_radius_m=1.0,  # Acceptance radius (meters)
                yaw_deg=0.0,  # Optional, set to 0.0
                camera_photo_distance_m=0.0,  # Optional, set to 0.0
                camera_action=MissionItem.CameraAction.NONE,  # Correct MAVSDK Enum for camera action
                vehicle_action=MissionItem.VehicleAction.NONE,  # Correct MAVSDK Enum for vehicle action
            )
            ]
    print("Lat=", mission_items[0].latitude_deg)
    print("Long=", mission_items[0].longitude_deg)
    
Create_MissionItem(370000000, 1200000000)
