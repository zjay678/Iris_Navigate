#!/usr/bin/env python
"""
This is for Discover lab Iris+ Drone mission planning
Author: Zone Shi
Last Modified: May, 4, 2018
"""
import dronekit
import exceptions
import socket
import time

def Iris_Connnect(pi_serial, pi_rate):

    """connect with ardupilot via serial port"""

    try:
        """For Rasepberry pi """
        #Iris = dronekit.connect(pi_serial, wait_ready=True, baud=pi_rate)

        """For debugging with DroneKit-sitl"""
        #Iris = dronekit.connect('tcp:127.0.0.1:5760',wait_ready=True)

        """For debugging with sim_vehicle"""
        Iris = dronekit.connect('127.0.0.1:14550',wait_ready=True)
    # Bad TCP connection
    except socket.error:
        print('No server exists!')

    # Bad TTY connection
    except exceptions.OSError as e:
        print('No serial exists!')

    # API Error
    except dronekit.APIException:
        print('Timeout!')

    # Other error
    except:
        print('Some other error!')

    return Iris

class Irismission(object):

    def __init__(self, vehicle):

        self.vehicle = vehicle

    def arm_and_takeoff(self, targetAlt = 1):
        """
        Arms vehicle and fly to a TargetAltitude.
        """

        print("Pre-arm checks")
        # Don't try to arm until autopilot is ready
        while not self.vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)

        print("Arming motors")
        # Copter should arm in GUIDED mode
        self.vehicle.mode    = dronekit.VehicleMode("GUIDED")
        self.vehicle.armed   = True

        # Confirm vehicle armed before attempting to take off
        while not self.vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(1)

        print("Taking off!")
        self.vehicle.simple_takeoff(targetAlt) # Take off to target altitude

        # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
        #  after Vehicle.simple_takeoff will execute immediately)
        while True:
            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
            #Break and return from function just below target altitude.
            if self.vehicle.location.global_relative_frame.alt>=targetAlt*0.95:
                print("Reached target altitude")
                break
            time.sleep(1)

    def Go_to_Target(self, point, airspeed=2.0):
        """
        Iris go to a give point with specified airspeed.
        """
        self.vehicle.airspeed = airspeed
        self.vehicle.simple_goto(point)
        time.sleep(30)

    def Check_list(self):
        # vehicle is an instance of the Vehicle class
        print("Autopilot Firmware version: %s" % self.vehicle.version)
        print("Autopilot capabilities (supports ftp): %s" % self.vehicle.capabilities.ftp)
        print("Global Location: %s" % self.vehicle.location.global_frame)
        print("Global Location (relative altitude): %s" % self.vehicle.location.global_relative_frame)
        print("Local Location: %s" % self.vehicle.location.local_frame)   #NED
        print("Attitude: %s" % self.vehicle.attitude)
        print("Velocity: %s" % self.vehicle.velocity)
        print("GPS: %s" % self.vehicle.gps_0)
        print("Groundspeed: %s" % self.vehicle.groundspeed)
        print("Airspeed: %s" % self.vehicle.airspeed)
        print("Battery: %s" % self.vehicle.battery)
        #print("Rangefinder: %s" % self.vehicle.rangefinder)
        #print("Rangefinder distance: %s" % self.vehicle.rangefinder.distance)
        #print("Rangefinder voltage: %s" % self.vehicle.rangefinder.voltage)
        print("Heading: %s" % self.vehicle.heading)
        print("Is Armable?: %s" % self.vehicle.is_armable)
        print("System status: %s" % self.vehicle.system_status.state)
        print("Mode: %s" % self.vehicle.mode.name)    # settable
        print("Armed: %s" % self.vehicle.armed)    # settable

    def loop(self):
        """Performs positioning and displays/exports the results."""
        pass

if __name__ == "__main__":

    pi_serial_addr = '/dev/ttyAMA0'
    pi_serial_rate = 57600

    Iris_Vehicle = Iris_Connnect(pi_serial_addr,pi_serial_rate)

    r = Irismission(Iris_Vehicle)
    #Pre-flight checking
    #r.Check_list()
    #Change to GUIDED mode and takeoff to target altitude
    r.arm_and_takeoff(targetAlt = 10)
    Iris_point1 = dronekit.LocationGlobalRelative(-35.361354, 149.165218,15)
    r.Go_to_Target(Iris_point1)
    while True:
        r.loop()
