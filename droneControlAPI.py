# This API is used to control the drone through Python and DroneAPI in MAVProxy. Please see the Google Doc for documentation on what these functions do.


def takeoff():
    vehicle.mode = VehicleMode("LOITER")
    vehicle.armed = True
    sleep(1)
    vehicle.channel_override = {"3":2000}
    sleep(3)
    vehicle.channel_override = {"3":1500}
    return;
#comment

def land():
    vehicle.mode = VehicleMode("LAND")
    releaseControllAll()
    return;

def altDecV( pcnt ):
    vehicle.channel_override = {"3":(1500-(pcnt*5))}
    return;

def altDecP():
    vehicle.channel_override = {"3":1250}
    sleep(0.5)
    vehicle.channel_override = {"3":1500}
    return;

def altIncV( pcnt ):
    vehicle.channel_override = {"3":(1500+(pcnt*5))}
    return;

def altIncP():
    vehicle.channel_override = {"3":1750}
    sleep(0.5)
    vehicle.channel_override = {"3":1500}
    return;

def leftV( pcnt ):
    vehicle.channel_override = {"1":(1500-(pcnt*5))}
    return;
   
def leftP():
    vehicle.channel_override = {"1":1250}
    sleep(0.5)
    vehicle.channel_override = {"1":1500}
    return;

def rightV( pcnt ):
    vehicle.channel_override = {"1":(1500+(pcnt*5))}
    return;

def rightP():
    vehicle.channel_override = {"1":1750}
    sleep(0.5)
    vehicle.channel_override = {"1":1500}
    return;

def forwardV( pcnt ):
    vehicle.channel_override = {"2":(1500+(pcnt*5))}
    return;

def forwardP():
    vehicle.channel_override = {"2":1750}
    sleep(0.5)
    vehicle.channel_override = {"2":1500}
    return;

def reverseV( pcnt ):
    vehicle.channel_override = {"2":(1500-(pcnt*5))}
    return;

def reverseP():
    vehicle.channel_override = {"2":1250}
    sleep(0.5)
    vehicle.channel_override = {"2":1500}
    return;

def spinLeftV( pcnt ):
    vehicle.channel_override = {"4":(1500-(pcnt*5))}
    return;

def spinLeftP():
    vehicle.channel_override = {"4":1250}
    sleep(0.5)
    vehicle.channel_override = {"4":1500}
    return;

def spinRightV( pcnt ):
    vehicle.channel_override = {"4":(1500+(pcnt*5))}
    return;

def spinRightP():
    vehicle.channel_override = {"4":1750}
    sleep(0.5)
    vehicle.channel_override = {"4":1500}
    return;

def releaseControlAlt():
    vehicle.channel_override = {"3":0}
    return;

def releaseControlYaw():
    vehicle.channel_override = {"4":0}
    return;

def releaseControlSide():
    vehicle.channel_override = {"1":0}
    return;

def releaseControlFB():
    vehicle.channel_override = {"2":0}
    return;

def releaseControlAll():
    vehicle.channel_override = {"1":0, "2":0, "3":0, "4":0}
    return;

def getAltitude():

    return;

def getHeading():

    return;

def getAngle():

    return;

def getGPSPosition():

    return;


# GPS COORDINATES
# Input GPS coordinates, set mode to auto
