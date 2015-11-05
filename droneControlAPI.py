﻿# This API is used to control the drone through Python and DroneAPI in MAVProxy. Please see the Google Doc for documentation on what these functions do.

import time
from droneControlAPI.lib import VehicleMode, Locations, Command
from pymavlink import mavutil


class mikeObject:
    def __init__(self):
        #initialize vehicle
        api = local_connect()
        self.vehicle = api.get_vehicles()[0]
        return;

    def takeoff(self):
        self.vehicle.mode = VehicleMode("LOITER")
        self.vehicle.armed = True
        time.sleep(1)
        self.vehicle.channel_override = {"3":2000}
        time.sleep(3)
        self.vehicle.channel_override = {"3":1500}
        return;

    def armTest(self):
        self.vehicle.armed = True
        time.sleep(8)
        self.vehicle.armed = False
        return;

    def land(self):
        self.vehicle.mode = VehicleMode("LAND")
        releaseControllAll()
        return;

    def hover(self):
        self.vehicle.mode = VehicleMode("LOITER")
        self.vehicle.channel_override = {"1":1500, "2":1500, "3":1500, "4":1500}
        return;

    def altDecV(self, pcnt ):
        self.vehicle.channel_override = {"3":(1500-(pcnt*5))}
        return;

    def altDecP(self):
        self.vehicle.channel_override = {"3":1250}
        time.sleep(0.5)
        self.vehicle.channel_override = {"3":1500}
        return;

    def altIncV(self, pcnt ):
        self.vehicle.channel_override = {"3":(1500+(pcnt*5))}
        return;

    def altIncP(self):
        self.vehicle.channel_override = {"3":1750}
        time.sleep(0.5)
        self.vehicle.channel_override = {"3":1500}
        return;

    def leftV(self, pcnt ):
        self.vehicle.channel_override = {"1":(1500-(pcnt*5))}
        return;
   
    def leftP(self):
        self.vehicle.channel_override = {"1":1250}
        time.sleep(0.5)
        self.vehicle.channel_override = {"1":1500}
        return;

    def rightV(self, pcnt ):
        self.vehicle.channel_override = {"1":(1500+(pcnt*5))}
        return;

    def rightP(self):
        self.vehicle.channel_override = {"1":1750}
        time.sleep(0.5)
        self.vehicle.channel_override = {"1":1500}
        return;

    def forwardV(self, pcnt ):
        self.vehicle.channel_override = {"2":(1500+(pcnt*5))}
        return;

    def forwardP(self):
        self.vehicle.channel_override = {"2":1750}
        time.sleep(0.5)
        self.vehicle.channel_override = {"2":1500}
        return;

    def reverseV(self, pcnt ):
        self.vehicle.channel_override = {"2":(1500-(pcnt*5))}
        return;

    def reverseP(self):
        self.vehicle.channel_override = {"2":1250}
        time.sleep(0.5)
        self.vehicle.channel_override = {"2":1500}
        return;

    def spinLeftV(self, pcnt ):
        self.vehicle.channel_override = {"4":(1500-(pcnt*5))}
        return;

    def spinLeftP(self):
        self.vehicle.channel_override = {"4":1250}
        time.sleep(0.5)
        self.vehicle.channel_override = {"4":1500}
        return;

    def spinRightV(self, pcnt ):
        self.vehicle.channel_override = {"4":(1500+(pcnt*5))}
        return;

    def spinRightP(self):
        self.vehicle.channel_override = {"4":1750}
        time.sleep(0.5)
        self.vehicle.channel_override = {"4":1500}
        return;

    def releaseControlAlt(self):
        self.vehicle.channel_override = {"3":0}
        return;

    def releaseControlYaw(self):
        self.vehicle.channel_override = {"4":0}
        return;

    def releaseControlSide(self):
        self.vehicle.channel_override = {"1":0}
        return;

    def releaseControlFB(self):
        self.vehicle.channel_override = {"2":0}
        return;

    def releaseControlAll(self):
        self.vehicle.channel_override = {"1":0, "2":0, "3":0, "4":0}
        return;

    def getAltitude(self):
        altitude = self.vehicle.location.global_frame.alt
        return altitude;

    def getHeading(self):

        return;

    def getAngle(self):
        angle = self.vehicle.attitude.roll
        return angle;

    def getGPSPosition(self):
        latidude = self.vehicle.location.global_frame.lat
        longitude = self.vehicle.location.global_frame.lon
        altitude = self.vehicle.location.global_frame.alt
        return (latitude, longitude, altitude);


    # GPS COORDINATES
    # Input GPS coordinates, set mode to auto