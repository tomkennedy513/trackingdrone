# DO NOT TOUCH THIS SRSLY OMG BUT ACTUALLY

import sys
import os
import time
from droneapi.lib import VehicleMode, Location, Command
from pymavlink import mavutil
from dronekit import connect
sys.path.append("/home/shan/code/trackingdrone/")
from droneControlAPI import mikeObject

api=local_connect()
vehicle = api.get_vehicles()[0]

drone = mikeObject()

#DO NOT TOUCH ANYTHING ABOVE SO HELP ME GOD.....


drone.releaseControlAll(vehicle)

print "Starting Takeoff"
drone.takeoff(vehicle)
print "Takeoff!!!!"
time.sleep(5)

drone.rightP(vehicle)
time.sleep(1)
drone.rightP(vehicle)
time.sleep(1)
drone.rightP(vehicle)
time.sleep(1)
drone.rightP(vehicle)
time.sleep(1)
drone.rightP(vehicle)
time.sleep(1)

print "RELEASE CONTROL"
drone.releaseControlAll(vehicle)

time.sleep(15)

drone.releaseControlAll(vehicle)

print "Start Landing"
drone.land(vehicle)

time.sleep(30)


print "Disarm!"
drone.disarm(vehicle)
