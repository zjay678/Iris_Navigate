#!/usr/bin/env python
"""
This is for Discover lab Iris+ Drone localization
"""
from time import sleep
from time import time as pi_time
from pypozyx import (POZYX_POS_ALG_UWB_ONLY,POZYX_POS_ALG_TRACKING, POZYX_3D, Coordinates,PositionError,DeviceRange, POZYX_SUCCESS, POZYX_ANCHOR_SEL_AUTO,
                     DeviceCoordinates, PozyxSerial, Data,get_first_pozyx_serial_port, SingleRegister, DeviceList)
from pythonosc.udp_client import SimpleUDPClient
import struct
import serial

"""Ardupilot message header and id"""
MSG_HEADER = 0x01
MSGID_BEACON_CONFIG = 0x02
MSGID_BEACON_DIST = 0x03
MSGID_POSITION = 0x04
CONFIG_TX_GAIN = 33.0

class ReadyToLocalize(object):
    """Continuously calls the Pozyx positioning function and prints its position."""

    loop_start = 0
    stage = 0  # 0 = initialisation, 1 = normal flight
    beacon_sent_count = 0
    beacon_sent_time = 0
    counter = 0

    def __init__(self, pozyx, pi_serial, osc_udp_client, anchors, algorithm=POZYX_POS_ALG_UWB_ONLY, dimension=POZYX_3D, height=1000, remote_id=None):
        self.pozyx = pozyx
        self.pi_serial = pi_serial   #configure the pi's output serial port
        self.osc_udp_client = osc_udp_client
        self.anchors = anchors
        self.algorithm = algorithm
        self.dimension = dimension
        self.height = height
        self.remote_id = remote_id

    def setup(self):
        """Sets up the Pozyx for positioning by calibrating its anchor list."""
        self.pozyx.clearDevices(self.remote_id)
        self.setAnchorsManual()
        self.printPublishConfigurationResult()

        sleep(5)

    def loop(self):
        """Performs positioning and displays/exports the results."""

        self.counter += 1 #slow down counter

        """initialise start time"""
        if self.loop_start ==0:
            self.loop_start = self.millis()

        """advance to normal flight stage after 1 minitues."""
        if self.stage ==0:
            time_diff = self.millis()-self.loop_start
            if time_diff > 60000:
                self.stage = 1
                print("Stage1")

        """slow down counter"""
        if self.counter>=20:
            self.counter = 0

        """during stage 0 (init) send position and beacon config as quickly as possible
        during stage 1 send about every 2 seconds
        """
        if (self.stage==0) or (self.counter ==0):
            self.send_beacon_config()
            self.get_position()
            if (self.beacon_sent_count>0) and (self.beacon_sent_time!=0):
                time_diff = self.millis()-self.beacon_sent_time
                hz = self.beacon_sent_count/(time_diff/1000.0)
                print("Beacon Hz: {}".format(hz))
            self.beacon_sent_count = 0
            self.beacon_sent_time = self.millis()

        """send beacon distances"""
        self.get_ranges()
        self.beacon_sent_count +=1


    def millis(self):
        return int(round(pi_time() * 1000)) #just follow Arduno's style, time keeper in python


    def send_vehicle_position(self,position,pos_err):
        """Sends the Pozyx's position to Ardupilot via Serial Port."""
        if position.x ==0 and position.y==0:
            return

        msg = [position.x,position.y,position.z,0]
        
        data = struct.pack('<iiih',msg[0],msg[1],msg[2],msg[3])
        self.send_message(MSGID_POSITION,14,data)

    def get_position(self):
        """Get position of tag."""
        position = Coordinates()
        pos_err = PositionError()

        status_pos = self.pozyx.doPositioning(
            position, self.dimension, self.height, self.algorithm, remote_id=self.remote_id)

        if status_pos == POZYX_SUCCESS:
            status_err = self.pozyx.getPositionError(pos_err,remote_id=self.remote_id)
            if status_err == POZYX_SUCCESS:
                self.printPublishPosition(position) #display position on remote computer
                print(position)
                self.send_vehicle_position(position,pos_err)     #send to ardupilot

        else:
            self.printPublishErrorCode("positioning")


    def send_message(self,msg_id,data_len,data_buf):

        msg_len = data_len+1

        checksum = 0
        checksum ^= msg_id
        checksum ^= msg_len

        for data in data_buf:
            checksum ^= data

        output = struct.pack('<B',MSG_HEADER)
        output += struct.pack('<B',msg_id)
        output += struct.pack('<B',msg_len)
        output += data_buf
        output += struct.pack('<B',checksum)
        self.pi_serial.write(output)
        self.pi_serial.flush()

    def send_beacon_config(self):
        """send all beacon config to ardupilot"""
        i = 0
        for anchor in self.anchors:
            msg = [i,4,int(anchor.pos.x), int(anchor.pos.y), int(anchor.pos.z)]

            data = struct.pack('<BBiii',msg[0],msg[1],msg[2],msg[3],msg[4])

            self.send_message(MSGID_BEACON_CONFIG,14,data)
            i +=1

        print("Sent anchor info")

    def get_ranges(self):
        range = DeviceRange();
        success = False
        for idx,anchor in enumerate(self.anchors):
            status = self.pozyx.doRanging(anchor.network_id,range,self.remote_id)
            if status == POZYX_SUCCESS:
                self.send_beacon_distance(idx,range.distance)
                success = True
        if success==False:
            print("Failed to get any ranges")

    def send_beacon_distance(self,id,distance):
        msg = [id,distance]
        data = struct.pack('<BI',msg[0],msg[1])
        self.send_message(MSGID_BEACON_DIST,5,data)

    def configure_beacons(self):
        configured_ok = True
        if self.set_device_gain(self.remote_id,CONFIG_TX_GAIN)==False:
            configured_ok = False

        for anchor in self.anchors:
            if self.set_device_gain(anchor.network_id,CONFIG_TX_GAIN)==False:
                configured_ok = False

        return configured_ok

    def set_device_gain(self,dev_id,gain):
        tx_power = Data([0], 'f')
        gain_ok = False
        retry = 0
        while (not gain_ok) and (retry<5):
            if self.pozyx.getTxPower(tx_power,dev_id)==POZYX_SUCCESS:
                if tx_power.data!=gain:
                    self.pozyx.setTxPower(CONFIG_TX_GAIN,dev_id)
                else:
                    gain_ok = True

            retry +=1

        return gain_ok

        """                     split line                   """
    def printPublishPosition(self, position):
        """Prints the Pozyx's position and possibly sends it as a OSC packet"""
        network_id = self.remote_id
        if network_id is None:
            network_id = 0
        print("POS ID {}, x(mm): {pos.x} y(mm): {pos.y} z(mm): {pos.z}".format(
            "0x%0.4x" % network_id, pos=position))
        if self.osc_udp_client is not None:
            self.osc_udp_client.send_message(
                "/position", [network_id, int(position.x), int(position.y), int(position.z)])

    def printPublishErrorCode(self, operation):
        """Prints the Pozyx's error and possibly sends it as a OSC packet"""
        error_code = SingleRegister()
        network_id = self.remote_id
        if network_id is None:
            self.pozyx.getErrorCode(error_code)
            print("LOCAL ERROR %s, %s" % (operation, self.pozyx.getErrorMessage(error_code)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message("/error", [operation, 0, error_code[0]])
            return
        status = self.pozyx.getErrorCode(error_code, self.remote_id)
        if status == POZYX_SUCCESS:
            print("ERROR %s on ID %s, %s" %
                  (operation, "0x%0.4x" % network_id, self.pozyx.getErrorMessage(error_code)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message(
                    "/error", [operation, network_id, error_code[0]])
        else:
            self.pozyx.getErrorCode(error_code)
            print("ERROR %s, couldn't retrieve remote error code, LOCAL ERROR %s" %
                  (operation, self.pozyx.getErrorMessage(error_code)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message("/error", [operation, 0, -1])
            # should only happen when not being able to communicate with a remote Pozyx.

    def setAnchorsManual(self):
        """Adds the manually measured anchors to the Pozyx's device list one for one."""
        status = self.pozyx.clearDevices(self.remote_id)
        for anchor in self.anchors:
            status &= self.pozyx.addDevice(anchor, self.remote_id)
        if len(self.anchors) > 4:
            status &= self.pozyx.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO, len(self.anchors))
        return status

    def printPublishConfigurationResult(self):
        """Prints and potentially publishes the anchor configuration result in a human-readable way."""
        list_size = SingleRegister()

        self.pozyx.getDeviceListSize(list_size, self.remote_id)
        print("List size: {0}".format(list_size[0]))
        if list_size[0] != len(self.anchors):
            self.printPublishErrorCode("configuration")
            return
        device_list = DeviceList(list_size=list_size[0])
        self.pozyx.getDeviceIds(device_list, self.remote_id)
        print("Calibration result:")
        print("Anchors found: {0}".format(list_size[0]))
        print("Anchor IDs: ", device_list)

        for i in range(list_size[0]):
            anchor_coordinates = Coordinates()
            self.pozyx.getDeviceCoordinates(device_list[i], anchor_coordinates, self.remote_id)
            print("ANCHOR, 0x%0.4x, %s" % (device_list[i], str(anchor_coordinates)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message(
                    "/anchor", [device_list[i], int(anchor_coordinates.x), int(anchor_coordinates.y), int(anchor_coordinates.z)])
                sleep(0.025)

    def printPublishAnchorConfiguration(self):
        """Prints and potentially publishes the anchor configuration"""
        for anchor in self.anchors:
            print("ANCHOR,0x%0.4x,%s" % (anchor.network_id, str(anchor.coordinates)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message(
                    "/anchor", [anchor.network_id, int(anchor.coordinates.x), int(anchor.coordinates.y), int(anchor.coordinates.z)])
                sleep(0.025)


if __name__ == "__main__":
    # shortcut to not have to find out the port yourself
    serial_port = get_first_pozyx_serial_port()

    #pi_serial_addr = '/dev/pts/19'     #linux vserial port, debug use
    pi_serial_addr = '/dev/ttyAMA0'
    pi_serial_rate = 115200
    if serial_port is None:
        print("No Pozyx connected. Check your USB cable or your driver!")
        quit()

    remote_id = 0x6977                 # remote device network ID
    remote = False                   # whether to use a remote device
    if not remote:
        remote_id = None

    use_processing = True             # enable to send position data through OSC
    ip = "127.0.0.1"                   # IP for the OSC UDP
    network_port = 8888                # network port for the OSC UDP
    osc_udp_client = None

    if use_processing:
        osc_udp_client = SimpleUDPClient(ip, network_port)
    # necessary data for calibration, change the IDs and coordinates yourself
    anchors = [DeviceCoordinates(0x6e07, 1, Coordinates(0, 0, -1200)),
               DeviceCoordinates(0x6e51, 1, Coordinates(4780, 0, -1200)),
               DeviceCoordinates(0x6e5e, 1, Coordinates(0, 4070, -1200)),
               DeviceCoordinates(0x6e5f, 1, Coordinates(4780, 4070, -1200))]

    algorithm = POZYX_POS_ALG_TRACKING  # positioning algorithm to use
    dimension = POZYX_3D                # positioning dimension
    height = 0                          # height of device, required in 2.5D positioning

    pozyx = PozyxSerial(serial_port)

    pi_serial = serial.Serial(port = pi_serial_addr,baudrate = pi_serial_rate) #initialize Raspberry's serial port

    r = ReadyToLocalize(pozyx, pi_serial, osc_udp_client, anchors, algorithm, dimension, height, remote_id)
    r.setup()
    while True:
        r.loop()
