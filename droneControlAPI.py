# This API is used to control the drone through Python and DroneAPI in MAVProxy. Please see the Google Doc for documentation on what these functions do.


def takeoff():
    vehicle.mode = VehicleMode("LOITER")
    vehicle.armed = True
    sleep(1)
    vehicle.channel_override = {"3":2000}
    sleep(3)
    vehicle.channel_override = {"3":1500}
    return;


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

def leftV(int):
    
    return;
   
def leftP():

    return;

def rightV(int):

    return;

def rightP():

    return;

def forwardV(int):

    return;

def forwardP():

    return;

def reverseV(int):

    return;

def reverseP():

    return;

def spinLeftV(int):

    return;

def spinLeftP():

    return;

def spinRightV(int):

    return;

def spinRightP():

    return;

def releaseControlAlt():

    return;

def releaseControlYaw():

    return;

def releaseControlSide():

    return;

def releaseControlFB():

    return;

def releaseControlAll():

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
