# This API is used to control the drone through Python and DroneAPI in MAVProxy. Please see the Google Doc for documentation on what these functions do.

import time
from droneapi.lib import VehicleMode, Location, Command
from pymavlink import mavutil
from dronekit import connect


class mikeObject:
    def __init__(self, vehicle):
        #initialize vehicle
        #api=local_connect()
        #vehicle = api.get_vehicles()[0]
        #vehicle = connect("/dev/ttyAMA0", "115200")
        print "INITIALIZING"
        return;

    def takeoff(self, vehicle):
        vehicle.mode = VehicleMode("LOITER")
        time.sleep(2)
        vehicle.armed = True
        print "overriding"
        vehicle.channel_override = {'3':2000}
        print "done"
        time.sleep(3)
        vehicle.channel_override = {"3":1500}
        return;

    def arm(self, vehicle):
        print "ARMING VEHICLE"
        vehicle.armed = True
        print "VEHICLE SHOULD BE ARMED"
        return;

    def disarm(self, vehicle):
        print "DISARMING VEHICLE"
        vehicle.armed = False
        print "VEHICLE SHOULD BE DISARMED"
        return;

    def testVel(self, vehicle):
        vehicle.mode = VehicleMode("GUIDED")
        vehicle.armed = True
        time.sleep(1)
        channel_override = {"3":2000}
        time.sleep(3)
        channel_override = {"3":1500}
        return;

    def land(self, vehicle):
        vehicle.mode = VehicleMode("LAND")
        releaseControllAll()
        return;

    def hover(self, vehicle):
        vehicle.mode = VehicleMode("LOITER")
        vehicle.channel_override = {"1":1500, "2":1500, "3":1500, "4":1500}
        return;

    def altDecV(self, vehicle, pcnt ):
        vehicle.channel_override = {"3":(1500-(pcnt*5))}
        return;

    def altDecP(self, vehicle):
        vehicle.channel_override = {"3":1250}
        time.sleep(0.5)
        vehicle.channel_override = {"3":1500}
        return;

    def altIncV(self, vehicle, pcnt ):
        channel_override = {"3":(1500+(pcnt*5))}
        return;

    def altIncP(self, vehicle):
        vehicle.channel_override = {"3":1750}
        time.sleep(0.5)
        vehicle.channel_override = {"3":1500}
        return;

    def leftV(self, vehicle, pcnt ):
        vehicle.channel_override = {"1":(1500-(pcnt*5))}
        return;
   
    def leftP(self, vehicle):
        vehicle.channel_override = {"1":1250}
        time.sleep(0.5)
        vehicle.channel_override = {"1":1500}
        return;

    def rightV(self, vehicle, pcnt ):
        vehicle.channel_override = {"1":(1500+(pcnt*5))}
        return;

    def rightP(self, vehicle):
        vehicle.channel_override = {"1":1750}
        time.sleep(0.5)
        vehicle.channel_override = {"1":1500}
        return;

    def forwardV(self, vehicle, pcnt ):
        vehicle.channel_override = {"2":(1500+(pcnt*5))}
        return;

    def forwardP(self, vehicle):
        vehicle.channel_override = {"2":1750}
        time.sleep(0.5)
        vehicle.channel_override = {"2":1500}
        return;

    def reverseV(self, vehicle, pcnt ):
        vehicle.channel_override = {"2":(1500-(pcnt*5))}
        return;

    def reverseP(self, vehicle):
        vehicle.channel_override = {"2":1250}
        time.sleep(0.5)
        vehicle.channel_override = {"2":1500}
        return;

    def spinLeftV(self, vehicle, pcnt ):
        vehicle.channel_override = {"4":(1500-(pcnt*5))}
        return;

    def spinLeftP(self, vehicle):
        vehicle.channel_override = {"4":1250}
        time.sleep(0.5)
        vehicle.channel_override = {"4":1500}
        return;

    def spinRightV(self, vehicle, pcnt ):
        vehicle.channel_override = {"4":(1500+(pcnt*5))}
        return;

    def spinRightP(self, vehicle):
        vehicle.channel_override = {"4":1750}
        time.sleep(0.5)
        vehicle.channel_override = {"4":1500}
        return;

    def releaseControlAlt(self, vehicle):
        vehicle.channel_override = {"3":0}
        return;

    def releaseControlYaw(self, vehicle):
        vehicle.channel_override = {"4":0}
        return;

    def releaseControlSide(self, vehicle):
        vehicle.channel_override = {"1":0}
        return;

    def releaseControlFB(self, vehicle):
        vehicle.channel_override = {"2":0}
        return;

    def releaseControlAll(self, vehicle):
        vehicle.channel_override = {"1":0, "2":0, "3":0, "4":0}
        return;

    def getAltitude(self, vehicle):
        altitude = vehicle.location.global_frame.alt
        return altitude;

    def getHeading(self, vehicle):

        return;

    def getAngle(self, vehicle):
        angle = vehicle.attitude.roll
        return angle;

    def getGPSPosition(self, vehicle):
        latidude = vehicle.location.global_frame.lat
        longitude = vehicle.location.global_frame.lon
        altitude = vehicle.location.global_frame.alt
        return (latitude, longitude, altitude);
#
#    def send_ned_velocity(velocity_x, velocity_y, velocity_z):
#        msg = vehicle.message_factory.set_position_target_local_ned_encode(
#        0,       # time_boot_ms (not used)
#        0, 0,    # target system, target component
#        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
#        0b0000111111000111, # type_mask (only speeds enabled)
#        0, 0, 0, # x, y, z positions (not used)
#        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
#        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
#        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
#        send command to vehicle
#        vehicle.send_mavlink(msg)
#	     return;

    # GPS COORDINATES
    # Input GPS coordinates, set mode to auto
