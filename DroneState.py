#!/usr/bin/python
# DroneStateMachine\DroneState.py
# State Machine pattern using 'if' statements
# to determine the next state.
import string, sys
''''sys.path += ['..\DroneStateMachine']
from State import State
from StateMachine import StateMachine
from DroneAction import DroneAction'''
import Tkinter as tk
import multiprocessing
import time
from queue import Queue
from threading import Thread




import numpy as np
import cv2
import pid
import droneControlAPI

frame = None
roiPts = []
inputMode = False
xPts = []
yPts = []


def selectROI(event, x, y, flags, param):
    global frame, roiPts, inputMode, yPts, xPts
    if inputMode and event == cv2.EVENT_LBUTTONDOWN and len(roiPts) < 4:
        roiPts.append((x, y))
        xPts.append(x)
        yPts.append(y)
        cv2.circle(frame, (x, y), 4, (255, 204, 0), 2)
        cv2.imshow("frame", frame)
        print (roiPts)
        if len(roiPts) == 4:
            cv2.circle(frame, ((sum(xPts) / 4), (sum(yPts) / 4)), 1, (255, 0, 0), 2)
            cv2.imshow("frame", frame)


def tom():
    global frame, roiPts, inputMode
    camera = cv2.VideoCapture(0)
    xPid = pid.PID(.1,.1,.1)
    yPid = pid.PID(.1,.1,.1)

    cv2.namedWindow("frame")
    #cv2.namedWindow("HSV")
    #cv2.namedWindow("Back Projection")

    cv2.setMouseCallback("frame", selectROI)
    termination = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 0)
    roiBox = None
    while True:
        (grabbed, frame) = camera.read()
        if not grabbed:
            break
        if roiBox is not None:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            #cv2.imshow("HSV", hsv)
            backProj = cv2.calcBackProject([hsv], [0], roiHist, [0, 180], 1)
            #cv2.imshow("Back Projection", backProj)

            (r, roiBox) = cv2.meanShift(backProj, roiBox, termination)
            if not r:
                print ("error")
                cv2.putText(frame, "Object Out of Frame!", (7, 25), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 2)
            else:
                (r, roiBox) = cv2.CamShift(backProj, roiBox, termination)
                pts = np.int0(cv2.cv.BoxPoints(r))
            # print pts
                cPtx = (pts[0][0] + pts[1][0] + pts[2][0] + pts[3][0]) / 4
                cPty = (pts[0][1] + pts[1][1] + pts[2][1] + pts[3][1]) / 4
                cPt = (cPtx, cPty)
                boxArea = .5 * (
                (pts[0][0] * pts[1][1]) + (pts[1][0] * pts[2][1]) + (pts[2][0] * pts[3][1]) + (pts[3][0] * pts[0][1]) - (
                pts[1][0] * pts[0][1]) - (pts[2][0] * pts[1][1]) - (pts[3][0] * pts[2][1]) - (pts[0][0] * pts[3][1]))
                print (cPt)
                cv2.polylines(frame, [pts], True, (255, 204, 0), 2)
                #cv2.circle(frame, cPt, 1, (255, 0, 0), 2)
                #cv2.putText(frame, 'target', (cPtx, (cPty - 7)), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)
                #cv2.putText(frame, str(cPt), (7, 25), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0), 2)
                #cv2.putText(frame, str(boxArea), (7, 55), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0), 2)

                print (camera.get(3))
                print (camera.get(4))

                print (xPid.GenOut(cPtx-camera.get(3)/2))
                print (yPid.GenOut(cPty-camera.get(4)/2))
        cv2.imshow("frame", frame)
        key = cv2.waitKey(1) & 0xFF

        if key == 32 and len(roiPts) < 4:
            inputMode = True
            orig = frame.copy()

            while len(roiPts) < 4:
                cv2.imshow("frame", frame)
                cv2.waitKey(0)

            roiPts = np.array(roiPts)
            s = roiPts.sum(axis=1)
            tl = roiPts[np.argmin(s)]
            br = roiPts[np.argmax(s)]
            print (roiPts)

            roi = orig[tl[1]:br[1], tl[0]:br[0]]
            roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            # roi = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)

            roiHist = cv2.calcHist([roi], [0], None, [16], [0, 180])
            roiHist = cv2.normalize(roiHist, roiHist, 0, 255, cv2.NORM_MINMAX)

            roiBox = (tl[0], tl[1], br[0], br[1])
            print (roiBox)
        # choose new target without resetting
        elif key == ord("r"):
            frame = None
            roiPts = []
            inputMode = False
            xPts = []
            yPts = []
        elif key == ord("q"):
            break


    camera.release()
    cv2.destroyAllWindows()















































































class ObjTrckThrd(Thread):
    def __init__(self):
        Thread.__init__(self)

    def run(self):
        while True:
            for x in range(0, 20):
                print("Thread is running: ", x)
            time.sleep(5)

    def stop(self):
        self._stop.set()

    def stopped(self):
        return self._stop.isSet()



def ObjTrckPrcs():
    """Mock Object Tracking process"""
    for x in range(0, 20):
        print("The Object Tracking Process is running: ", x)
        time.sleep( 5 )
    return
def LineTrckPrcs():
    """Mock Line Tracking process"""
    p = multiprocessing.current_process()
    print ('Starting:', p.name, p.pid)
    sys.stdout.flush()
    print ('hello') 
    for x in range(0, 20):
        print("The Line Tracking Process is running: ", x)
    time.sleep( 10 )
    print ('Exiting :', p.name, p.pid)
    sys.stdout.flush()


class LineTrckPrcss(multiprocessing.Process):

    def run(self):
        print ('In %s' % self.name)
        return
        

def GPSTrckPrcs():
    """Mock GPS waypoitns process"""
    for x in range(0, 20):
        print("The Object Tracking Process is running: ", x)
        time.sleep( 5 )
    return


class State:
    def run(self):
        assert 0, "run not implemented"
    def next(self, input):
        assert 0, "next not implemented"

class StateMachine:
    def __init__(self, initialState):
        self.currentState = initialState
        self.currentState.run()
    # Template method:
    def runAll(self, inputs):
        for i in inputs:
            self.currentState = self.currentState.next(i)
            self.currentState.run()
    def run(self, input, lati=None, longi=None):
        self.currentState = self.currentState.next(input)
        self.currentState.run()

class DroneAction:
    def __init__(self, action):
        self.action = action
    def __str__(self): return self.action
    def __cmp__(self, other):
        return cmp(self.action, other.action)
    # Necessary when __cmp__ or __eq__ is defined
    # in order to make this class usable as a
    # dictionary key:
    def __hash__(self):
        return hash(self.action)

# Static fields; an enumeration of instances:
DroneAction.ready = DroneAction("Drone Ready")
DroneAction.takeoff = DroneAction("Automated Takeoff")
DroneAction.land = DroneAction("Automated Landing")
DroneAction.eland = DroneAction("Emergency Landing")
DroneAction.trackLine = DroneAction("Track Line")
DroneAction.trackObject = DroneAction("Track Object")
DroneAction.end = DroneAction("End Tracking")
DroneAction.error = DroneAction("Error!")
DroneAction.lost = DroneAction("Lost!")
DroneAction.gps = DroneAction("GPS Waypoints")

# A different subclass for each state:

class Boot(State):
    def run(self):
        '''Need to run proper startup script blah blah blah'''
        print("Boot Sequence: Please Wait...")

    def next(self, input):
        '''Probs want to go to Ready automatically...'''
        '''Maybe first wait till everything is good... How to check??'''
        if input == DroneAction.ready:
            return DroneState.ready
        return DroneState.boot

class Ready(State):
    def run(self):
        '''Any continous checks to make??'''
        print("Okay ready, LMK when to takeoff")

    def next(self, input):
        if input == DroneAction.takeoff:
            '''drone.takeoff(vehicle)'''
            print('Will takeoff now')
            return DroneState.hover
        return DroneState.ready

class Hover(State):
    def run(self):
        '''Any continous checks to make??'''
        print("Yay I'm Hovering, LMK what to do")

    def next(self, input):
        if input == DroneAction.eland:
            '''drone.land(vehicle)'''
            print('This is an Emergency Landing!')
            return DroneState.ready
        if input == DroneAction.land:
            '''drone.rtl(vehicle)'''
            '''This is for when returning back to takeoff location... Will not work indoors!!'''
            print('Standard landing')
            return DroneState.ready
        if input == DroneAction.trackLine:
            print('Will follow the line')
            return DroneState.trackLine
        if input == DroneAction.trackObject:
            print('Will track the object')
            return DroneState.trackObject
        if input == DroneAction.gps:
            print('Will go to GPS waypoints')
            return DroneState.gps
        return DroneState.hover

class TrackLine(State):
    p = None

    def run(self):
        '''Run proper CV process'''
        '''while(True):
            print("Look I'm Following a Line!!") '''
        print("Look I'm Following a Line!!")
        global p
        p = multiprocessing.Process(name='LineTrckPrcs', target=LineTrckPrcs)
        p.LineTrckPrcs = True
        p.start()

    def next(self, input):
        if input == DroneAction.error:
            '''drone.hover(vehicle)'''
            print('Error!!')
            p.terminate()
            p.join()
            return DroneState.hover
        if input == DroneAction.end:
            '''drone.hover(vehicle)'''
            print('Will stop tracking now')
            print ('DURING:', p, p.pid, p.is_alive())
            p.terminate()
            print ('TERMINATED:', p, p.pid, p.is_alive())
            p.join()
            print ('JOINED:', p, p.pid, p.is_alive())
            return DroneState.hover
        if input == DroneAction.lost:
            drone.hover(vehicle)
            print('Oh no, I lost it :(' )
            p.terminate()
            p.join()
            return DroneState.hover
        return DroneState.trackLine

class TrackObject(State):

    '''p = multiprocessing.Process(target=ObjTrckPrcs)'''
    worker = None

    def run(self):
        '''Run proper CV process'''
        print("Look I'm Tracking an Object :)")
        '''p.start()'''
        global worker
        worker = ObjTrckThrd()
        # Setting daemon to True will let the main thread exit even though the workers are blocking
        worker.daemon = True
        worker.start()


    def next(self, input):
        worker.stopped()
        if input == DroneAction.error:
            '''drone.hover(vehicle)'''
            print('Error!!')
            '''p.terminate()'''
            return DroneState.hover
        if input == DroneAction.end:
            '''drone.hover(vehicle)'''
            print('Will stop tracking now')
            '''p.terminate()'''
            return DroneState.hover
        if input == DroneAction.lost:
            '''drone.hover(vehicle)'''
            print('Oh no, I lost it :(')
            '''p.terminate()'''
            return DroneState.hover
        return DroneState.trackObject

class GPSWaypoints(State):

    p = multiprocessing.Process(target=GPSTrckPrcs)

    def run(self, lati, longi):
        ''' GPS API ??? '''
        ''' API: getGPSPosition ''' 
        '''drone.AddWaypoint(vehicle, lati, longi)'''
        '''drone.ExecuteWapoints(vehicle)'''
        print("Going towards GPS waypoints :) ", lati, " & ", longi)
        p.start()

    def next(self, input):
        if input == DroneAction.error:
            '''drone.hover(vehicle)'''
            print('Error!!')
            p.terminate()
            return DroneState.hover
        if input == DroneAction.end:
            ''''drone.hover(vehicle)'''
            print('Oh nvm, will hover now')
            p.terminate()
            return DroneState.hover
        return DroneState.gps

class DroneState(StateMachine):
    def __init__(self):
        # Initial state
        StateMachine.__init__(self, DroneState.boot)


# Static variable initialization:
DroneState.boot = Boot()
DroneState.ready = Ready()
DroneState.hover = Hover()
DroneState.trackLine = TrackLine()
DroneState.trackObject = TrackObject()
DroneState.gps = GPSWaypoints()

DroneInstance = DroneState()

class Example(tk.Frame):
    def __init__(self, parent):
        tk.Frame.__init__(self, parent)

        # create a prompt, an input box, an output label,
        # and a button to do the computation
        self.prompt = tk.Label(self, text="Select Operation Mode", anchor="w")

        self.prompt1 = tk.Label(self, text="Latitude:", anchor="w")
        self.prompt2 = tk.Label(self, text="Longitude:", anchor="w")
        self.entry1 = tk.Entry(self)
        self.entry2 = tk.Entry(self)
        self.output = tk.Label(self, text="")
        self.gps = tk.Button(self, text="Go To GPS Waypoints", command = lambda: self.OnButtonClick(10))

        self.ready = tk.Button(self, text="Drone Ready", command = lambda: self.OnButtonClick(1))
        self.takeoff = tk.Button(self, text="Automated Takeoff", command = lambda: self.OnButtonClick(2))
        self.land = tk.Button(self, text="Automated Landing", command = lambda: self.OnButtonClick(3))
        self.eland = tk.Button(self, text="Emergency Landing", command = lambda: self.OnButtonClick(4))
        self.trackLine = tk.Button(self, text="Track Line", command = lambda: self.OnButtonClick(5))
        self.trackObject = tk.Button(self, text="Track Object", command = lambda: self.OnButtonClick(6))
        self.end = tk.Button(self, text="End Tracking", command = lambda: self.OnButtonClick(7))
        self.error = tk.Button(self, text="Error!", command = lambda: self.OnButtonClick(8))
        self.lost = tk.Button(self, text="Lost!", command = lambda: self.OnButtonClick(9))

        # lay the widgets out on the screen. 
        self.prompt.pack(side="top", fill="x")
        self.pack(side = "top")
        self.prompt1.pack(side="top", fill="x")
        self.entry1.pack(side="top", fill="x", padx=20)
        self.prompt2.pack(side="top", fill="x")
        self.entry2.pack(side="top", fill="x", padx=20)
        self.output.pack(side="top", fill="x", expand=True)

        self.gps.pack(side="top")
        self.ready.pack(side="top")
        self.takeoff.pack(side="top")
        self.land.pack(side="top")
        self.eland.pack(side="top")
        self.trackLine.pack(side="top")
        self.trackObject.pack(side="top")
        self.end.pack(side="top")
        self.error.pack(side="top")
        self.lost.pack(side="top")


    def OnButtonClick(self, mode):
        if mode == 1:
            DroneInstance.run(DroneAction.ready)
        elif mode == 2:
            DroneInstance.run(DroneAction.takeoff)
        elif mode == 3:
            DroneInstance.run(DroneAction.land)
        elif mode == 4:
            DroneInstance.run(DroneAction.eland)
        elif mode == 5:
            DroneInstance.run(DroneAction.trackLine)
        elif mode == 6:
            DroneInstance.run(DroneAction.trackObject)
        elif mode == 7:
            DroneInstance.run(DroneAction.end)
        elif mode == 8:
            DroneInstance.run(DroneAction.error)
        elif mode == 9:
            DroneInstance.run(DroneAction.lost)
        elif mode == 10:
            try:
                lati = int(self.entry1.get())
                longi = int(self.entry2.get())
                print("The waypoints entered were ", lati, " & ", longi)
                DroneInstance.run(DroneAction.gps)
            except ValueError:
                result = "Please enter digits only"
                self.output.configure(text=result)

# if this is run as a program (versus being imported),
# create a root window and an instance of our example,
# then start the event loop

if __name__ == "__main__":
    root = tk.Tk()
    Example(root).pack(fill="both", expand=True)
    root.mainloop()