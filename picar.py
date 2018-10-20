
import time

from threading import Thread, Event, Semaphore, Timer
try:
    from Queue import Queue
except:
    from queue import Queue
from myclapp import MyCLApp
from optparse import make_option
from pprint import pprint

import random

from math import asin, pi

from copy import deepcopy as copy

## import RPi.GPIO as GPIO
import pigpio
mypi = pigpio.pi()

# ---------------------------------------------------------------------------------
class Servos:

    # -----------------------------------------------------------------------------
    def __init__(self,
                 pan=13,pan_range=[1200,1800],pan_calib=[1565.,13.],
                 tilt=12,tilt_range=[750,1900],tilt_calib=[1630.,13.]):

        ## 1565.0, 1630.0
        
        self.pan = pan
        self.pan_range = pan_range
        self.pan_calib = pan_calib
        self.tilt = tilt
        self.tilt_range = tilt_range
        self.tilt_calib = tilt_calib

        mypi.set_mode(self.pan,pigpio.ALT0)
        mypi.set_mode(self.tilt,pigpio.ALT0)

        mypi.set_PWM_frequency(self.pan,50.)
        mypi.set_PWM_frequency(self.tilt,50.)

        self.home = (1500,1500)
        self.goto(None,None)
        time.sleep(0.1)
        self.goto(0.,0.)
        
    # -----------------------------------------------------------------------------
    def goto(self,pan,tilt):

        print("goto",pan,tilt)

        if pan != None and tilt != None:
            pan =  self.pan_calib[0] + pan * self.pan_calib[1]
            pan = min(max(pan,self.pan_range[0]),self.pan_range[1])
            
            tilt =  self.tilt_calib[0] + tilt * self.tilt_calib[1]
            tilt = min(max(tilt,self.tilt_range[0]),self.tilt_range[1])
        else:
            pan,tilt = self.home
            
        print("goto -->",pan,tilt)

        mypi.set_servo_pulsewidth(self.pan,pan)
        mypi.set_servo_pulsewidth(self.tilt,tilt)

        ## time.sleep(0.01)

        self.timer = Timer(2.,self.pwm_off)
        self.timer.start()
        
        self.pos = ((pan-self.pan_calib[0])/self.pan_calib[1],(tilt-self.tilt_calib[0])/self.tilt_calib[1])
        print(self.pos)
        
    # -----------------------------------------------------------------------------
    def move_by(self,deltaPan,deltaTilt):
        self.goto( self.pos[0]+deltaPan, self.pos[1]+deltaTilt ) 

    # -----------------------------------------------------------------------------
    def pwm_off(self):
        print("turning pwm off")
        mypi.set_servo_pulsewidth(self.pan,0)
        mypi.set_servo_pulsewidth(self.tilt,0)

        
# ---------------------------------------------------------------------------------
class Motors:

    # -----------------------------------------------------------------------------
    def __init__(self,in1=18,in2=23,in3=24,in4=25,degToTime=1.1/90.,record=False):# ,degToTime=0.635/90.):

        self.in1 = in1
        self.in2 = in2
        self.in3 = in3
        self.in4 = in4
        self.degToTime = degToTime
        
        ## GPIO.setup(self.in1, GPIO.OUT)
        ## GPIO.setup(self.in2, GPIO.OUT)
        ## GPIO.setup(self.in3, GPIO.OUT)
        ## GPIO.setup(self.in4, GPIO.OUT)
        mypi.set_mode(self.in1, pigpio.OUTPUT)
        mypi.set_mode(self.in2, pigpio.OUTPUT)
        mypi.set_mode(self.in3, pigpio.OUTPUT)
        mypi.set_mode(self.in4, pigpio.OUTPUT)

        self.can_move = Event()        
        self.turning = Event()        
        self.moving = Queue()
        self.record = record
        if self.record:
            self.records = Queue()
        self.turning.clear()
        self.can_move.set()
        self.stop()

    # -----------------------------------------------------------------------------
    def __del__(self):
        mypi.set_mode(self.in1, pigpio.INPUT)
        mypi.set_mode(self.in2, pigpio.INPUT)
        mypi.set_mode(self.in3, pigpio.INPUT)
        mypi.set_mode(self.in4, pigpio.INPUT)
        
        
    # -----------------------------------------------------------------------------
    def sleep(self,tsleep):
        time.sleep(tsleep)
        
    # -----------------------------------------------------------------------------
    def send(self,in1,in2,in3,in4):
        mypi.write(self.in1, in1)
        mypi.write(self.in2, in2)
        mypi.write(self.in3, in3)
        mypi.write(self.in4, in4)
        
    # -----------------------------------------------------------------------------
    def brake(self):
        if not self.turning.is_set():
            self.send(False,False,False,False)
        self.can_move.clear()
        if self.record and not self.turning.is_set():
            self.records.put( (time.time(),[0.,0.,0.,0.,1.]) )
        
    # -----------------------------------------------------------------------------
    def unbrake(self):
        if not self.can_move.is_set():
            self.can_move.set()
            self.direction()
        if self.record and not self.turning.is_set():
            fwd = float(self.direction == self.forward)
            self.records.put( (time.time(),[fwd,1.-fwd,0.,0.,0.]) )
        
    # -----------------------------------------------------------------------------
    def reverse(self):
        if self.can_move.is_set():
            self.send(False,True,False,True)
        ## else:
        ##     self.send(False,False,False,False)
        self.moving.put(True)
        self.direction = self.reverse
        if self.record:
            self.records.put( (time.time(),[0.,1.,0.,0.,0.]) )
        
    # -----------------------------------------------------------------------------
    def forward(self):
        if self.can_move.is_set():
            self.send(True,False,True,False)
        ## else:
        ##     self.send(False,False,False,False)
        self.moving.put(True)
        self.direction = self.forward
        if self.record:
            self.records.put( (time.time(),[1.,0.,0.,0.,0.]) )
        
    # -----------------------------------------------------------------------------
    def turn_right(self,angle=None):
        self.turning.set()
        self.send(False,True,True,False)
        if angle:
            t1 = time.time()
            self.sleep(angle*self.degToTime)
            t2 = time.time()
            self.direction()
        self.turning.clear()
        if self.record:
            fwd = float(self.direction == self.forward)
            self.records.put( (t1,[fwd,1.-fwd,angle*self.degToTime,0.,0.]) )
            self.records.put( (t2,[fwd,1.-fwd,angle*self.degToTime,0.,0.]) )

    # -----------------------------------------------------------------------------
    def turn_left(self,angle=None):
        self.turning.set()
        self.send(True,False,False,True)
        if angle:
            t1 = time.time()
            self.sleep(angle*self.degToTime)
            t2 = time.time()
            self.direction()
        self.turning.clear()
        if self.record:
            fwd = float(self.direction == self.forward)
            self.records.put( (t1,[fwd,1.-fwd,0.,angle*self.degToTime,0.]) )
            self.records.put( (t2,[fwd,1.-fwd,0.,angle*self.degToTime,0.]) )
            
        
    # -----------------------------------------------------------------------------
    def stop(self):
        self.send(False,False,False,False)
        self.moving.put(False)
        # self.moving.clear()
        self.direction = self.stop
        if self.record:
            self.records.put( (time.time(),[0.,0.,0.,0.,1.]) )



# ---------------------------------------------------------------------------------
class Sensors:

    ## def __init__(self,trg=26,echos=[13,16]):
    # -----------------------------------------------------------------------------
    ## def __init__(self,trg=10,echos=[9,11]):
    def __init__(self,trg=10,front=[17,27,22],back=[9,11,5]):#echos=[17,27,22,9,11,5]):
        self.trg=trg

        ## GPIO.setup(self.trg, GPIO.OUT)
        mypi.set_mode(self.trg, pigpio.OUTPUT)

        ## self.distances=[ Sensors.TimeToDistance(pin, sens_id) for sens_id,pin in enumerate(echos) ]
        self.distances=[ Sensors.TimeToDistance(pin, sens_id) for sens_id,pin in enumerate(front+back) ]
        self.front = self.distances[:len(front)]
        self.back  = self.distances[len(front):]
        self.last = None
        self.lock = Semaphore(1)

    def __del__(self):
        mypi.set_mode(self.trg, pigpio.INPUT)
        
    # -----------------------------------------------------------------------------
    class TimeToDistance:

        def __init__(self,echo,sens_id):
            self.echo=echo
            self.sens_id=sens_id
            ## GPIO.setup(self.echo, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            mypi.set_mode(self.echo, pigpio.INPUT)
            mypi.set_pull_up_down(self.echo, pigpio.PUD_UP)
            
            self.readings = []
            
        def read(self,time):
            ## val = GPIO.input(self.echo)
            val = mypi.read(self.echo)
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
    def run(self,raw=False):

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
            
        ## GPIO.output(self.trg, True)
        mypi.write(self.trg, True)
        time.sleep(0.0001)
        t0 = time.time()
        t1 = t0
        ## GPIO.output(self.trg, False)
        mypi.write(self.trg, False)
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

        ## distances = [ dist.measure(t0) for dist in self.distances ]
        front = [ dist.measure(t0) for dist in self.front ]
        back  = [ dist.measure(t0) for dist in self.back ]
        self.lock.release()
        ## return distances
        if raw:
            return t0,front,back
        else:
            return [ min(front),min(back) ]


# ---------------------------------------------------------------------------------
class Pilot:

    # -----------------------------------------------------------------------------
    def __init__(self,car,flip_rate=0.05,turn_rate=0.6,step=0.5):

        self.car = car
        self.flip_cdf = flip_rate
        self.turn_cdf = (turn_rate+flip_rate)
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
                         make_option("-T","--turn-step",action="store",dest="turn_step",
                                     type="float",default=30.),
                         make_option("-S","--servos-step",action="store",dest="servos_step",
                                     type="float",default=5.),
                         make_option("-s","--safe-distance",action="store",
                                     dest="safe_distance",type="float",default=25.),
                         make_option("-d","--dump-metrics",action="store_true",
                                     dest="dump_metrics",default=False),
                         make_option("-v","--verbose",action="store_true",
                                     dest="verbose",default=False),
                     ]
        )

        ## GPIO.setmode(GPIO.BCM)
        self.camera = None
        self.motors = None
        self.sensors = None
        self.servos = None
        self.mon = None
        self.doquit = Event()
        self.pilot = Pilot(self)

        
    ## # -----------------------------------------------------------------------------
    ## def __del__(self):
    ##     ## GPIO.cleanup()
    ##     if self.camera:
    ##         del self.camera
            
    # -----------------------------------------------------------------------------
    def run(self):
        self.load_config()
        pprint( self.options_ ) 
        pprint( self.args_ )
            
        self.turnStep = self.options_.turn_step
        self.motors = Motors(record=self.options_.dump_metrics)

        self.safeDistance = self.options_.safe_distance
        self.sensors = Sensors()

        self.servosStep = self.options_.servos_step
        self.servos = Servos()

        self.dumpMetrics = self.options_.dump_metrics
        if self.dumpMetrics:
            self.dout = None
            self.mout = None
            self.distance_records = Queue()
            self.dump = Thread(target=self.dump_metrics)
            self.dump.start()
            ## self.dump = Timer(2.,self.dump_metrics)
            ## self.dump.daemon = True
            ## self.dump.start()
            
        self.camera = None
        if self.options_.enable_camera:
            ## from camera import VideoCamera
            ## self.camera = VideoCamera(classify=True,fmrate=60)
            from camera_pi import Camera
            self.camera = Camera() ## VideoCamera(classify=True,fmrate=60)

        self.mon = Thread(target=self.monitor)
        self.doquit.clear()
        self.mon.start()

    # -----------------------------------------------------------------------------
    def closest_object(self,direction):
        raw = self.sensors.run(raw=True)
        self.time = raw[0]
        self.raw_distances = copy(raw[1:])
        self.distances = list(map(min,self.raw_distances))
        towards = self.distances[direction]
        return towards

    # -----------------------------------------------------------------------------
    def dump_metrics(self):
        while not self.doquit.is_set():
            if self.dout is None:
                self.dout = open('distance_records.csv','w+')
                self.dout.write('timestamp,')
                self.dout.write(','.join( 'front%d' % ii for ii in range(len(self.sensors.front))  ) )
                self.dout.write(',')
                self.dout.write(','.join( 'back%d' % ii for ii in range(len(self.sensors.back))  ) )
                self.dout.write('\n')
                self.dout.flush()
            
            if self.mout is None:
                self.mout = open('motors.csv','w+')
                self.mout.write('timestamp,front,reverse,turn_left,turn_right,stop\n')
                self.mout.flush()

            ## print('reading distances')
            time_stamps = []
            records = []
            while ( not self.distance_records.empty() ):
                time_stamp, record = self.distance_records.get()
                time_stamps.append(time_stamp)
                records.append(record)
            for time_stamp, record in zip(time_stamps,records):
                strings = [ str(time_stamp) ] + [ str(front) for front in record[0] ] + [ str(back) for back in record[1] ] 
                self.dout.write(','.join(strings))
                self.dout.write('\n')
            self.dout.flush()
            ## print(time_stamps,record)

            ## print('reading motors')
            time_stamps = []
            records = []
            while ( not self.motors.records.empty() ):
                time_stamp, record = self.motors.records.get()
                time_stamps.append(time_stamp)
                records.append(record)
            for time_stamp, record in zip(time_stamps,records):
                strings = [ str(time_stamp) ] + [ str(idir) for idir in record ]
                self.mout.write(','.join(strings))
                self.mout.write('\n')
            self.mout.flush()

            
            time.sleep(10.)
            
    # -----------------------------------------------------------------------------
    def monitor(self):
        moving = self.motors.moving.get()
        while not self.doquit.is_set():
            while ( not self.motors.moving.empty() ) or ( not moving ):
                moving = self.motors.moving.get()
            direction = 0 if self.motors.direction == self.motors.forward else 1
            nchecks = 2
            nsafe = 0
            for check in xrange(nchecks):
                towards = self.closest_object(direction)
                if self.dumpMetrics:
                    self.distance_records.put( (self.time,self.raw_distances) )
                if towards > self.safeDistance:
                    nsafe += 1
                else:
                    nbelow = reduce(lambda x,y: x+y, map(lambda x: x<self.safeDistance, self.raw_distances))
                    if nbelow > 1:
                        nsafe = 0
                        break
                    if self.options_.verbose:
                        print("Unsafe :", self.raw_distances)
            if nsafe < 1:
                self.brake()
                if self.motors.can_move.is_set():
                    if self.options_.verbose:
                        print("Distances :", self.raw_distances)
            else:
                if not self.motors.can_move.is_set():
                    if self.options_.verbose:
                        print("Distances: ", self.raw_distances)
                self.unbrake()
                
    # -----------------------------------------------------------------------------
    def quit(self):
        self.doquit.set()
        self.motors.moving.put(None)
        self.mon.join()
        
    # -----------------------------------------------------------------------------
    def forward(self):
        self.motors.forward()
        
    # -----------------------------------------------------------------------------
    def reverse(self):
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

        
    # -----------------------------------------------------------------------------
    def camCentre(self):
        self.servos.goto(0.,0.)

    # -----------------------------------------------------------------------------
    def panL(self):
        self.servos.move_by(self.servosStep,0.)

    # -----------------------------------------------------------------------------
    def panR(self):
        self.servos.move_by(-self.servosStep,0.)

    # -----------------------------------------------------------------------------
    def tiltU(self):
        self.servos.move_by(0.,-self.servosStep)

    # -----------------------------------------------------------------------------
    def tiltD(self):
        self.servos.move_by(0,+self.servosStep)
