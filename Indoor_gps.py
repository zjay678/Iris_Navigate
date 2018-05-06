"""
This is for Discover lab Iris+ Drone Indoor GPS with Optitrack
Author: Zone Shi
Last Modified: May, 4, 2018
GPS_TYPE,         14   <-- sets driver to MAV_GPS driver
SERIAL2_BAUD,     576  <-- set telemetry2 baud rate to 57600
SERIAL2_PROTOCOL, 1    <-- set telemetry2 protocol to mavlink
BRD_SER2_RTSCTS,  0    <-- turn off flow control
EK2_GPS_TYPE,     2    <-- only use 2D
"""
import dronekit
#from gwpy.time import to_gps
from datetime import datetime
import time


def send_fake_gps(vehicle,mocap_loca,mocap_vel):
    '''
        GPS sensor input message.  This is a raw sensor value sent by the GPS.
        This is NOT the global position estimate of the sytem.

        time_usec                 : Timestamp (micros since boot or Unix epoch) (uint64_t)
        gps_id                    : ID of the GPS for multiple GPS inputs (uint8_t)
        ignore_flags              : Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum).  All other fields must be provided. (uint16_t)
        time_week_ms              : GPS time (milliseconds from start of GPS week) (uint32_t)
        time_week                 : GPS week number (uint16_t)
        fix_type                  : 0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK (uint8_t)
        lat                       : Latitude (WGS84), in degrees * 1E7 (int32_t)
        lon                       : Longitude (WGS84), in degrees * 1E7 (int32_t)
        alt                       : Altitude (AMSL, not WGS84), in m (positive for up) (float)
        hdop                      : GPS HDOP horizontal dilution of position in m (float)
        vdop                      : GPS VDOP vertical dilution of position in m (float)
        vn                        : GPS velocity in m/s in NORTH direction in earth-fixed NED frame (float)
        ve                        : GPS velocity in m/s in EAST direction in earth-fixed NED frame (float)
        vd                        : GPS velocity in m/s in DOWN direction in earth-fixed NED frame (float)
        speed_accuracy            : GPS speed accuracy in m/s (float)
        horiz_accuracy            : GPS horizontal accuracy in m (float)
        vert_accuracy             : GPS vertical accuracy in m (float)
        satellites_visible        : Number of satellites visible. (uint8_t)
    '''
    time_usec = 0
    gps_id = 1
    ignore_flags = 32
    secsperweek = 604800
    time_week_ms = int((datetime.now()-datetime(1980,1,6)).total_seconds())#to_gps('now')
    time_week = time_week_ms/secsperweek
    fix_type = 3
    lat = mocap_loca[0]*1e7
    lon = mocap_loca[1]*1e7
    alt = mocap_loca[2]*1000
    hdop = 1.0
    vdop = 1.0
    vn = mocap_vel[0]
    ve = mocap_vel[0]
    vd = mocap_vel[0]
    speed_accuracy = 0.1
    horiz_accuracy = 0.1
    vert_accuracy = 0.1
    satellites_visible = 6

    vehicle.message_factory.gps_input_send(time_usec,gps_id,ignore_flags,time_week_ms,time_week,fix_type,lat,lon,alt,hdop,vdop,vn,ve,vd,speed_accuracy,horiz_accuracy,vert_accuracy,satellites_visible)

if __name__ == "__main__":

    pi_serial = '/dev/ttyAMA0'
    pi_rate = 57600

    mocap_loca = [41.698363326621,-86.23395438304738,100]
    mocap_vel = [0,0,0]

    """For Rasepberry pi """
    Iris = dronekit.connect(pi_serial, wait_ready=True, baud=pi_rate)

    while True:
        send_fake_gps(Iris,mocap_loca,mocap_vel)
        print("Position Send! \n")
        #print(GpsSecondsFromPyUTC(time.time())/secsInWeek)
        time.sleep(0.2)
