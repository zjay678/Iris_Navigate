"""
This is for Discover lab Iris+ Drone Indoor GPS with Optitrack
Author: Zone Shi
Last Modified: May, 4, 2018
GPS_TYPE,         14   <-- sets driver to MAV_GPS driver
SERIAL2_BAUD,     115  <-- set telemetry2 baud rate to 115200
SERIAL2_PROTOCOL, 1    <-- set telemetry2 protocol to mavlink
BRD_SER2_RTSCTS,  0    <-- turn off flow control
EK2_GPS_TYPE,     2    <-- only use 2D
"""
import dronekit
from glue.gpstime import GpsSecondsFromPyUTC,secsInWeek
import time

def send_fake_gps(vehicle,mocap_loca,mocap_vel):
    """
    Send SET_POSITION_TARGET_GLOBAL_INT command to request the vehicle fly to a specified LocationGlobal.
    """
    gps_time = GpsSecondsFromPyUTC(time.time())
    gps_week = gps_time/secsInWeek

    msg = vehicle.message_factory.gps_input_encode(
        0,  # timestamp
        1,  # gps_id
        1,  # ignore_flags
        gps_time,  # gps_time
        gps_week,  # gps week number
        3,  # fix_type
        mocap_loca.lat*1e7, # lat - X position in WGS84 frame in 1e7*meters
        mocap_loca.lon*1e7, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        mocap_loca.alt*1000, # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
        0.02,
        0.02,
        mocap_vel.velx, # X velocity in NED frame in m/s
        mocap_vel.vely, # Y velocity in NED frame in m/s
        mocap_vel.velz, # Z velocity in NED frame in m/s
        1, 5, 3, # vert_accuracy
        10) # satellite_visible
    # send command to vehicle
    vehicle.send_mavlink(msg)

if __name__ == "__main__":

    pi_serial = '/dev/ttyAMA0'
    pi_rate = 115200
    #mocap_loca.lat = 41.698363326621
    #mocap_loca.lon = -86.23395438304738
    #mocap_loca.alt = 100

    #mocap_vel.velx = 0
    #mocap_vel.vely = 0
    #mocap_vel.velz = 0
    """For Rasepberry pi """
    #Iris = dronekit.connect(pi_serial, wait_ready=True, baud=pi_rate)

    while True:
        #send_fake_gps(Iris,mocap_loca,mocap_vel)
        print(GpsSecondsFromPyUTC(time.time()))
        time.sleep(0.2)
