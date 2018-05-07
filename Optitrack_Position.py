import socket
import struct
from threading import Thread

def trace( *args ):
     pass#print( "".join(map(str,args)) )

# Create structs for reading various object types to speed up parsing.
Vector3 = struct.Struct( '<fff' )
Quaternion = struct.Struct( '<ffff' )
FloatValue = struct.Struct( '<f' )
DoubleValue = struct.Struct( '<d' )

class NatNetClient:
    def __init__( self ):
        # Change this value to the IP address of the NatNet server.
        self.serverIPAddress = "192.168.1.145"
        #self.serverIPAddress = ""
        # This should match the multicast address listed in Motive's streaming settings.
        self.multicastAddress = "239.255.42.99"

        # NatNet Command channel
        self.commandPort = 1510

        # NatNet Data channel
        self.dataPort = 1511

        # Set this to a callback method of your choice to receive per-rigid-body data at each frame.
        self.rigidBodyListener = None

        # NatNet stream version. This will be updated to the actual version the server is using during initialization.
        self.__natNetStreamVersion = (2,2,0,0)

    # Client/server message ids
    NAT_PING                  = 0
    NAT_PINGRESPONSE          = 1
    NAT_REQUEST               = 2
    NAT_RESPONSE              = 3
    NAT_REQUEST_MODELDEF      = 4
    NAT_MODELDEF              = 5
    NAT_REQUEST_FRAMEOFDATA   = 6
    NAT_FRAMEOFDATA           = 7
    NAT_MESSAGESTRING         = 8
    NAT_DISCONNECT            = 9
    NAT_UNRECOGNIZED_REQUEST  = 100

    # Create a data socket to attach to the NatNet stream
    def __createDataSocket( self, port ):
        result = socket.socket( socket.AF_INET,     # Internet
                              socket.SOCK_DGRAM,
                              socket.IPPROTO_UDP)    # UDP
        result.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        result.bind( ('', port) )

        mreq = struct.pack("4sl", socket.inet_aton(self.multicastAddress), socket.INADDR_ANY)
        result.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        return result

    # Unpack a rigid body object from a data packet
    def __unpackRigidBody( self, data ):
        offset = 0

        # ID (4 bytes)
        id = int.from_bytes( data[offset:offset+4], byteorder='little' )
        offset += 4
        trace( "ID:", id )

        # Position and orientation
        pos = Vector3.unpack( data[offset:offset+12] )
        offset += 12
        trace( "\tPosition:", pos[0],",", pos[1],",", pos[2] )
        rot = Quaternion.unpack( data[offset:offset+16] )
        offset += 16
        trace( "\tOrientation:", rot[0],",", rot[1],",", rot[2],",", rot[3] )

        # Send information to any listener.
        if self.rigidBodyListener is not None:
            self.rigidBodyListener( id, pos, rot )

        # RB Marker Data ( Before version 3.0.  After Version 3.0 Marker data is in description )
        if( self.__natNetStreamVersion[0] < 3 ) :
            # Marker count (4 bytes)
            markerCount = int.from_bytes( data[offset:offset+4], byteorder='little' )
            offset += 4
            markerCountRange = range( 0, markerCount )
            #trace( "\tMarker Count:", markerCount )

            # Marker positions
            for i in markerCountRange:
                pos = Vector3.unpack( data[offset:offset+12] )
                offset += 12
                #trace( "\tMarker", i, ":", pos[0],",", pos[1],",", pos[2] )

            if( self.__natNetStreamVersion[0] >= 2 ):
                # Marker ID's
                for i in markerCountRange:
                    id = int.from_bytes( data[offset:offset+4], byteorder='little' )
                    offset += 4
                    #trace( "\tMarker ID", i, ":", id )

                # Marker sizes
                for i in markerCountRange:
                    size = FloatValue.unpack( data[offset:offset+4] )
                    offset += 4
                    #trace( "\tMarker Size", i, ":", size[0] )

        # Skip padding inserted by the server
        offset += 4

        if( self.__natNetStreamVersion[0] >= 2 ):
            markerError, = FloatValue.unpack( data[offset:offset+4] )
            offset += 4
            #trace( "\tMarker Error:", markerError )

        return offset

    # Unpack data from a motion capture frame message
    def __unpackMocapData( self, data ):
        trace( "Begin MoCap Frame\n-----------------\n" )

        data = memoryview( data )
        offset = 0

        # Frame number (4 bytes)
        frameNumber = int.from_bytes( data[offset:offset+4], byteorder='little' )
        offset += 4
        trace( "Frame #:", frameNumber )

        # Marker set count (4 bytes)
        markerSetCount = int.from_bytes( data[offset:offset+4], byteorder='little' )
        offset += 4
        trace( "Marker Set Count:", markerSetCount )

        for i in range( 0, markerSetCount ):
            # Model name
            modelName, separator, remainder = bytes(data[offset:]).partition( b'\0' )
            offset += len( modelName ) + 1
            trace( "Model Name:", modelName.decode( 'utf-8' ) )

            # Marker count (4 bytes)
            markerCount = int.from_bytes( data[offset:offset+4], byteorder='little' )
            offset += 4
            trace( "Marker Count:", markerCount )

            for j in range( 0, markerCount ):
                pos = Vector3.unpack( data[offset:offset+12] )
                offset += 12
                #trace( "\tMarker", j, ":", pos[0],",", pos[1],",", pos[2] )

        # Unlabeled markers count (4 bytes)
        unlabeledMarkersCount = int.from_bytes( data[offset:offset+4], byteorder='little' )
        offset += 4
        trace( "Unlabeled Markers Count:", unlabeledMarkersCount )

        for i in range( 0, unlabeledMarkersCount ):
            pos = Vector3.unpack( data[offset:offset+12] )
            offset += 12
            #trace( "\tMarker", i, ":", pos[0],",", pos[1],",", pos[2] )

        # Rigid body count (4 bytes)
        rigidBodyCount = int.from_bytes( data[offset:offset+4], byteorder='little' )
        offset += 4
        trace( "Rigid Body Count:", rigidBodyCount )

        for i in range( 0, rigidBodyCount ):
            offset += self.__unpackRigidBody( data[offset:] )

    def __dataThreadFunction( self, socket ):
        while True:
            # Block for input
            data, addr = socket.recvfrom( 32768 ) # 32k byte buffer size
            if( len( data ) > 0 ):
                self.__processMessage( data )

    def __processMessage( self, data ):
        trace( "Begin Packet\n------------\n" )

        messageID = int.from_bytes( data[0:2], byteorder='little' )
        trace( "Message ID:", messageID )

        packetSize = int.from_bytes( data[2:4], byteorder='little' )
        trace( "Packet Size:", packetSize )

        offset = 4
        if( messageID == self.NAT_FRAMEOFDATA ):
            self.__unpackMocapData( data[offset:] )
        elif( messageID == self.NAT_UNRECOGNIZED_REQUEST ):
            trace( "Received 'Unrecognized request' from server" )
        elif( messageID == self.NAT_MESSAGESTRING ):
            message, separator, remainder = bytes(data[offset:]).partition( b'\0' )
            offset += len( message ) + 1
            trace( "Received message from server:", message.decode( 'utf-8' ) )
        else:
            trace( "ERROR: Unrecognized packet type" )

        trace( "End Packet\n----------\n" )

    def run( self ):
        # Create the data socket
        self.dataSocket = self.__createDataSocket( self.dataPort )
        if( self.dataSocket is None ):
            print( "Could not open data channel" )
            exit

        # Get data
        self.__dataThreadFunction(self.dataSocket)

def receiveRigidBodyFrame( id, position, rotation ):
    print( "Received frame for rigid body", id )
    print("Position",position)

if __name__ == '__main__':
    # This will create a new NatNet client
    streamingClient = NatNetClient()

    streamingClient.rigidBodyListener = receiveRigidBodyFrame
    streamingClient.run()
