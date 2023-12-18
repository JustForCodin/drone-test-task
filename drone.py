import time
from pymavlink import mavutil

# Connect to your copter
connection_string = '/dev/ttyUSB0'  # Change the port according to your requirements
vehicle = connect(connection_string, baud=57600, wait_ready=True)

# function to set height
def set_altitude(target_altitude):
    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        if current_altitude >= target_altitude - 1:
            break
        vehicle.simple_goto(
            (vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, target_altitude)
        )
        time.sleep(1)

# Take off!
def takeoff_and_goto(destination, target_altitude):
    print("Taking off...")
    vehicle.mode = VehicleMode("ALT_HOLD")
    vehicle.armed = True

    # Waiting till copter gains needed height
    set_altitude(target_altitude)
    print("Take off finished.")

    # Move the copter to point B
    print(f"Moving to the point {destination}...")
    vehicle.simple_goto(destination)

    # Waiting till it reaches B
    while True:
        distance_to_target = vehicle.location.global_relative_frame.distance_to(destination)
        if distance_to_target < 1:
            break
        time.sleep(1)

    print(f"Достигнута точка {destination}.")

# Rotate on given yaw
def rotate_to_azimuth(azimuth):
    print(f"Rotating on {azimuth} degrees...")
    msg = vehicle.message_factory.command_long_encode(
        0, 0,  
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, 
        0, 
        azimuth, 
        0,  
        1, 
        0, 0, 0 
    )
    vehicle.send_mavlink(msg)
    time.sleep(2) 

    print(f"Rotation on {azimuth} degrees finished.")

point_A = (50.450739, 30.461242)
point_B = (50.443326, 30.448078)
target_altitude = 100 

try:
    takeoff_and_goto(point_B, target_altitude)
    rotate_to_azimuth(350)

finally:
    print("Finishing...")
    vehicle.close()
