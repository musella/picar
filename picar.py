
import RPi.GPIO as GPIO

import time

from threading import Thread, Event
from myclapp import MyCLApp
from optparse import make_option
from pprint import pprint


# ---------------------------------------------------------------------------------
class Motors:

    def __init__(self,in1=27,in2=22,in3=5,in4=6,degToTime=0.64/90.):

        self.in1 = in1
        self.in2 = in2
        self.in3 = in3
        self.in4 = in4
        self.degToTime = degToTime
        
        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)
        GPIO.setup(self.in3, GPIO.OUT)
        GPIO.setup(self.in4, GPIO.OUT)

        self.moving = Event()
        self.stop()
        # self.direction = self.stop

    def sleep(self,tsleep):
        time.sleep(tsleep)
        
    def send(self,in1,in2,in3,in4):
        GPIO.output(self.in1, in1)
        GPIO.output(self.in2, in2)
        GPIO.output(self.in3, in3)
        GPIO.output(self.in4, in4)
        
    
    def reverse(self):
        self.send(False,True,False,True)
        self.moving.set()
        self.direction = self.reverse
        
    def forward(self):
        self.send(True,False,True,False)
        self.moving.set()
        self.direction = self.forward
        
    def turn_right(self,deg=90.):
        self.send(False,True,True,False)
        
    def turn_left(self,deg=90.):
        self.send(True,False,False,True)
        
    def stop(self):
        self.send(False,False,False,False)
        self.moving.clear()
        self.direction = self.stop



# ---------------------------------------------------------------------------------
class Sensors:

    def __init__(self,trg=26,echos=[13,16]):
        self.trg=trg

        GPIO.setup(self.trg, GPIO.OUT)

        self.distances=[ Sensors.TimeToDistance(echo) for echo in echos ]

        
    class TimeToDistance:

        def __init__(self,echo):
            self.echo=echo
            GPIO.setup(self.echo, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            self.done  = Event()

        def start(self):
            self.nosig = time.time()
            while GPIO.input(self.echo) == 1:
                self.sig = time.time()
            self.done.set()
            GPIO.remove_event_detect(self.echo)
        
        def arm(self):
            self.done.clear()
            GPIO.add_event_detect(self.echo, GPIO.RISING, callback=lambda x: self.start())
        
        def measure(self):
            self.done.wait()
        
            tl = self.sig - self.nosig
            self._distance = tl / 0.000058
            return self._distance

    def arm(self):
        for dist in self.distances:
            dist.arm()

    def measure(self):
        return [ dist.measure() for dist in self.distances ]
    

    def trigger(self):
        GPIO.output(self.trg, False)
        time.sleep(0.00001)

        GPIO.output(self.trg, True)
        time.sleep(0.00001)
        GPIO.output(self.trg, False)
        
    def run(self):
        
        self.arm()
        self.trigger()
        return self.measure()




# ---------------------------------------------------------------------------------
class PiCar(MyCLApp):
    
    def __init__(self):
        super(PiCar,self).__init__(option_list=[make_option("-k","--enable-camera",action="store_true",dest="enable_camera",
                                                           default=False),
                                               make_option("-t","--turn-step",action="store",dest="turn_step",type="float",
                                                           default=30.),
                                               make_option("-s","--safe-distance",action="store",dest="safe_distance",type="float",
                                                           default=30.),
                                           ]
        )

        GPIO.setmode(GPIO.BCM)
        self.camera = None
        self.motors = None
        self.sensors = None
        self.mon = None
        self.doquit = Event()
        self.done  = Event()
        
    def __del__(self):
        GPIO.cleanup()
        if self.camera:
            del self.camera
            
    def run(self):
        self.load_config()
        pprint( self.options_ ) 
        pprint( self.args_ )

        self.turnStep = self.options_.turn_step
        self.safeDistance = self.options_.safe_distance
        self.camera = None
        if self.options_.enable_camera:
            from camera import VideoCamera
            self.camera = VideoCamera(classify=True,fmrate=60)

            
        self.motors = Motors()
        self.sensors = Sensors()
        
        self.mon = Thread(target=self.monitor)
        self.doquit.clear()
        self.done.clear()
        self.mon.start()
        
    def monitor(self):
        while not self.doquit.is_set():
            self.motors.moving.wait()
            if self.doquit.is_set():
                self.done.set()
                return
            distances = self.sensors.run()
            towards = distances[0] if motors.direction == motors.forward else distances[1]
            if towards < self.safeDistance:
                self.stop()
        self.done.set()
        
    def quit(self):
        self.doquit.set()
        self.motors.moving.set()
        self.done.wait()
        
    def forward(self):
        self.motors.forward()
        
    def reverse(self):
        self.motors.reverse()
        
    def stop(self):
        self.motors.stop()

    def left(self):
        self.motors.turn_left()
        self.motors.sleep(self.turnStep*self.motors.degToTime)
        self.motors.direction()

    def right(self):
        self.motors.turn_right()
        self.motors.sleep(self.turnStep*self.motors.degToTime)
        self.motors.direction()
        
