import dronekit
import time
def set_home(vehicle):
    """
    set home location.
    """
    msg = vehicle.message_factory.att_pos_mocap_encode(
        10293,
        [1.0,0.0,0.0,0.0],0,0,0)
    # send command to vehicle
    vehicle.send_mavlink(msg)

vehicle = dronekit.connect('127.0.0.1:14550',wait_ready=True)
set_home(vehicle)
i = 0;
while True:
    set_home(vehicle)
    print(i)
    i +=1
    time.sleep(1)
