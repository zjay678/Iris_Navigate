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
#from glue.gpstime import GpsSecondsFromPyUTC,secsInWeek
import time

def send_fake_gps(vehicle,mocap_loca,mocap_vel):
    """
    Send SET_POSITION_TARGET_GLOBAL_INT command to request the vehicle fly to a specified LocationGlobal.
    """
    #gps_time = GpsSecondsFromPyUTC(time.time())
    #gps_week = gps_time/secsInWeek

    msg = vehicle.message_factory.set_home_position_encode(
        mocap_loca[0]*1e7, # lat - X position in WGS84 frame in 1e7*meters
        mocap_loca[1]*1e7, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        mocap_loca[2]*1000, # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
        0,0,0,[1.0,0,0,0],0,0,0) # satellite_visible
    # send command to vehicle
    vehicle.send_mavlink(msg)

if __name__ == "__main__":

    pi_serial = '/dev/ttyAMA0'
    pi_rate = 57600
    #mocap_loca.lat = 41.698363326621
    #mocap_loca.lon = -86.23395438304738
    #mocap_loca.alt = 100
    mocap_loca = [41.698363326621,-86.23395438304738,100]
    mocap_vel = [0,0,0]
    #mocap_vel.velx = 0
    #mocap_vel.vely = 0
    #mocap_vel.velz = 0
    """For Rasepberry pi """
    Iris = dronekit.connect(pi_serial, wait_ready=True, baud=pi_rate)

    while True:
        send_fake_gps(Iris,mocap_loca,mocap_vel)
        #print(GpsSecondsFromPyUTC(time.time())/secsInWeek)
        time.sleep(0.2)
