
import RPi.GPIO as GPIO

import time

from threading import Thread, Event, Semaphore
try:
    from Queue import Queue
except:
    from queue import Queue
from myclapp import MyCLApp
from optparse import make_option
from pprint import pprint

import random

from math import asin, pi

# ---------------------------------------------------------------------------------
class Motors:

    # -----------------------------------------------------------------------------
    def __init__(self,in1=18,in2=23,in3=24,in4=25,degToTime=0.635/90.):

        self.in1 = in1
        self.in2 = in2
        self.in3 = in3
        self.in4 = in4
        self.degToTime = degToTime
        
        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)
        GPIO.setup(self.in3, GPIO.OUT)
        GPIO.setup(self.in4, GPIO.OUT)

        self.can_move = Event()        
        self.turning = Event()        
        self.moving = Queue()
        self.turning.clear()
        self.can_move.set()
        self.stop()

        
    # -----------------------------------------------------------------------------
    def sleep(self,tsleep):
        time.sleep(tsleep)
        
    # -----------------------------------------------------------------------------
    def send(self,in1,in2,in3,in4):
        GPIO.output(self.in1, in1)
        GPIO.output(self.in2, in2)
        GPIO.output(self.in3, in3)
        GPIO.output(self.in4, in4)
        
    # -----------------------------------------------------------------------------
    def brake(self):
        if not self.turning.is_set():
            self.send(False,False,False,False)
        self.can_move.clear()

    # -----------------------------------------------------------------------------
    def unbrake(self):
        if not self.can_move.is_set():
            self.can_move.set()
            self.direction()
        
    # -----------------------------------------------------------------------------
    def reverse(self):
        if self.can_move.is_set():
            self.send(False,True,False,True)
        ## else:
        ##     self.send(False,False,False,False)
        self.moving.put(True)
        self.direction = self.reverse
        
    # -----------------------------------------------------------------------------
    def forward(self):
        if self.can_move.is_set():
            self.send(True,False,True,False)
        ## else:
        ##     self.send(False,False,False,False)
        self.moving.put(True)
        self.direction = self.forward
        
    # -----------------------------------------------------------------------------
    def turn_right(self,angle=None):
        self.turning.set()
        self.send(False,True,True,False)
        if angle:
            self.sleep(angle*self.degToTime)
            self.direction()
        self.turning.clear()

    # -----------------------------------------------------------------------------
    def turn_left(self,angle=None):
        self.turning.set()
        self.send(True,False,False,True)
        if angle:
            self.sleep(angle*self.degToTime)
            self.direction()
        self.turning.clear()
            
        
    # -----------------------------------------------------------------------------
    def stop(self):
        self.send(False,False,False,False)
        self.moving.put(False)
        # self.moving.clear()
        self.direction = self.stop



# ---------------------------------------------------------------------------------
class Sensors:

    ## def __init__(self,trg=26,echos=[13,16]):
    # -----------------------------------------------------------------------------
    def __init__(self,trg=10,echos=[9,11]):
        self.trg=trg

        GPIO.setup(self.trg, GPIO.OUT)

        self.distances=[ Sensors.TimeToDistance(pin, sens_id) for sens_id,pin in enumerate(echos) ]
        self.last = None
        self.lock = Semaphore(1)
        
    # -----------------------------------------------------------------------------
    class TimeToDistance:

        def __init__(self,echo,sens_id):
            self.echo=echo
            self.sens_id=sens_id
            GPIO.setup(self.echo, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            
            self.readings = []
            
        def read(self,time):
            val = GPIO.input(self.echo)
            self.readings.append( (time,val) )
            return val
        
        def arm(self):
            self.readings = []

        def measure(self,t0):
            if len(self.readings) < 2:
                return 0.
            if self.readings[0][1] == 1:
                sig = t0
            else:
                while len(self.readings) > 0 and self.readings[0][1] == 0:
                    sig = self.readings.pop(0)[0]

            nosig = sig
            while len(self.readings) > 0 and self.readings[0][1] == 1 :
                nosig = self.readings.pop(0)[0]
            tl = nosig - sig
            distance = tl * 340. * 0.5 * 100.
            return distance


    # -----------------------------------------------------------------------------
    def run(self):

        self.lock.acquire()

        for dist in self.distances:
            dist.arm()
        
        measDeltaT = 0.05
        minDeltaT = 2.*measDeltaT
        now = time.time()
        if self.last:
            deltaT = now - self.last
            if deltaT < minDeltaT:
                time.sleep(minDeltaT-deltaT)
        self.last = now
            
        GPIO.output(self.trg, True)
        time.sleep(0.0001)
        t0 = time.time()
        t1 = t0
        GPIO.output(self.trg, False)
        anyup = False
        while t1 - t0 < measDeltaT:
            vsum = 0
            for dist in self.distances:
                vsum += dist.read(t1)
            if not anyup:
                anyup = vsum > 0
            ## print(vsum,anyup)
            if anyup and vsum == 0: break
            t1 = time.time()

        distances = [ dist.measure(t0) for dist in self.distances ]
        self.lock.release()
        return distances


# ---------------------------------------------------------------------------------
class Pilot:

    # -----------------------------------------------------------------------------
    def __init__(self,car,flip_rate=0.2,turn_rate=0.6,step=0.5):

        self.car = car
        self.flip_cdf = flip_rate*step
        self.turn_cdf = (turn_rate+flip_rate)*step
        self.step = step

        self.doquit = Event()
        self.thread = None
        
    # -----------------------------------------------------------------------------
    def start(self):
        self.doquit.clear()
        self.thread = Thread(target=self.drive)
        self.thread.start()

    # -----------------------------------------------------------------------------
    def stop(self):
        ## print("stopping pilot")
        if self.thread:
            self.doquit.set()
            ## self.thread.join()
        self.thread = None

    # -----------------------------------------------------------------------------
    def direction(self):
        return 1 if self.car.motors.direction == self.car.motors.reverse else 0
        
    # -----------------------------------------------------------------------------
    def straight(self,flip=False):
        direction = self.direction()
        if flip:
            direction = 1-direction
        if direction == 0:
            self.car.motors.forward()
        else:
            self.car.motors.reverse()
        
    # -----------------------------------------------------------------------------
    def turn(self,angle):
        if angle > 0.:
            self.car.motors.turn_right(angle)
        elif angle < 0.:
            self.car.motors.turn_left(-angle)
                    
        
    # -----------------------------------------------------------------------------
    def drive(self):
        ## print("drive", self.doquit.is_set())
        radToDeg = 180. / pi
        while not self.doquit.is_set():
            
            flip = ( self.car.closest_object(self.direction()) < self.car.safeDistance )
            self.straight(flip)
            
            rnd = random.random()
            ## print("new step",rnd)
            if rnd < self.flip_cdf:
                ## print("flip direction")
                self.straight(True)
            elif rnd < self.turn_cdf:
                rnd /= self.turn_cdf
                angle = asin( 2.*rnd - 1. ) * radToDeg
                ## print("turning by", angle)
                self.turn(angle)
            
            self.doquit.wait(self.step)
            # time.sleep(self.step)
        ## print("stopped driving")
        
        
        
# ---------------------------------------------------------------------------------
class PiCar(MyCLApp):
    
    # -----------------------------------------------------------------------------
    def __init__(self):
        super(PiCar,self).__init__(
            option_list=[make_option("-k","--enable-camera",action="store_true",
                                     dest="enable_camera",default=False),
                         make_option("-t","--turn-step",action="store",dest="turn_step",
                                     type="float",default=30.),
                         make_option("-s","--safe-distance",action="store",
                                     dest="safe_distance",type="float",default=30.),
                     ]
        )

        GPIO.setmode(GPIO.BCM)
        self.camera = None
        self.motors = None
        self.sensors = None
        self.mon = None
        self.doquit = Event()
        self.done   = Event()
        self.pilot = Pilot(self)

        
    # -----------------------------------------------------------------------------
    def __del__(self):
        GPIO.cleanup()
        if self.camera:
            del self.camera
            
    # -----------------------------------------------------------------------------
    def run(self):
        self.load_config()
        pprint( self.options_ ) 
        pprint( self.args_ )

        self.turnStep = self.options_.turn_step
        self.safeDistance = self.options_.safe_distance
        self.camera = None
        if self.options_.enable_camera:
            ## from camera import VideoCamera
            ## self.camera = VideoCamera(classify=True,fmrate=60)
            from camera_pi import Camera
            self.camera = Camera() ## VideoCamera(classify=True,fmrate=60)
            
        self.motors = Motors()
        self.sensors = Sensors()
        
        self.mon = Thread(target=self.monitor)
        self.doquit.clear()
        self.done.clear()
        self.mon.start()

    # -----------------------------------------------------------------------------
    def closest_object(self,direction):
        self.distances = self.sensors.run()
        towards = self.distances[direction]
        return towards
    
    # -----------------------------------------------------------------------------
    def monitor(self):
        moving = self.motors.moving.get()
        while not self.doquit.is_set():
            while ( not self.motors.moving.empty() ) or ( not moving ):
                moving = self.motors.moving.get()
            direction = 0 if self.motors.direction == self.motors.forward else 1
            nchecks = 1
            nsafe = 0
            for check in xrange(nchecks):
                towards = self.closest_object(direction)
                if towards > self.safeDistance:
                    nsafe += 1
            if nsafe < 1:
                if self.motors.can_move.is_set():
                    print("Distances :", self.distances)
                self.brake()
            else:
                if not self.motors.can_move.is_set():
                    print("Distances: ", self.distances)
                self.unbrake()
                
        self.done.set()
        
    # -----------------------------------------------------------------------------
    def quit(self):
        self.doquit.set()
        self.motors.moving.put(None)
        self.done.wait()
        
    # -----------------------------------------------------------------------------
    def forward(self):
        # self.manualDrive()
        self.motors.forward()
        
    # -----------------------------------------------------------------------------
    def reverse(self):
        # self.manualDrive()
        self.motors.reverse()

    # -----------------------------------------------------------------------------
    def brake(self):
        self.motors.brake()

    # -----------------------------------------------------------------------------
    def unbrake(self):
        self.motors.unbrake()
        
    # -----------------------------------------------------------------------------
    def stop(self):
        self.motors.stop()
        self.manualDrive()
        self.motors.stop()

    # -----------------------------------------------------------------------------
    def left(self):
        self.motors.turn_left(self.turnStep)

    # -----------------------------------------------------------------------------
    def right(self):
        self.motors.turn_right(self.turnStep)
        
    # -----------------------------------------------------------------------------
    def leftL(self):
        self.motors.turn_left(90.)

    # -----------------------------------------------------------------------------
    def rightL(self):
        self.motors.turn_right(90.)

    # -----------------------------------------------------------------------------
    def leftU(self):
        self.motors.turn_left(180.)
        
    # -----------------------------------------------------------------------------
    def rightU(self):
        self.motors.turn_right(180.)

    # -----------------------------------------------------------------------------
    def selfDrive(self):
        self.pilot.start()

    # -----------------------------------------------------------------------------
    def manualDrive(self):
        if not self.pilot.doquit.is_set():
            self.pilot.stop()

        
