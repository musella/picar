import RPi.GPIO as gpio

import time

from threading import Event

class Motors:

    ## def __init__(self,in1=7,in2=11,in3=13,in4=15,tsleep=0.06):
    def __init__(self,in1=29,in2=31,in3=33,in4=37,degToTime=0.64/90.):

        self.in1 = in1
        self.in2 = in2
        self.in3 = in3
        self.in4 = in4
        self.degToTime = degToTime
        
        gpio.setmode(gpio.BOARD)
        gpio.cleanup()
        gpio.setup(self.in1, gpio.OUT)
        gpio.setup(self.in2, gpio.OUT)
        gpio.setup(self.in3, gpio.OUT)
        gpio.setup(self.in4, gpio.OUT)

        

    def __del__(self):
        gpio.cleanup()

    def sleep(self,tsleep):
        time.sleep(tsleep)
        
    def send(self,in1,in2,in3,in4):
        gpio.output(self.in1, in1)
        gpio.output(self.in2, in2)
        gpio.output(self.in3, in3)
        gpio.output(self.in4, in4)
        
    
    def reverse(self,sleep=1.5):
        self.send(False,True,False,True)
        self.sleep(sleep)
        self.stop()
        
    def forward(self,sleep=1.5):
        self.send(True,False,True,False)
        self.sleep(sleep)
        self.stop()
        
    def turn_right(self,deg=90.):
        self.send(False,True,True,False)
        self.sleep(deg*self.degToTime)
        self.stop()
        
    def turn_left(self,deg=90.):
        self.send(True,False,False,True)
        self.sleep(deg*self.degToTime)
        self.stop()
        
    def stop(self):
        self.send(False,False,False,False)
            
    
class Sensors:

    def __init__(self,trg=37,echos=[33,36]):
        self.trg=trg

        gpio.setmode(gpio.BOARD)
        gpio.setup(self.trg, gpio.OUT)

        self.distances=[ Sensors.TimeToDistance(echo) for echo in echos ]

        
    class TimeToDistance:

        def __init__(self,echo):
            self.echo=echo
            gpio.setup(self.echo, gpio.IN, pull_up_down=gpio.PUD_UP)
            self.done  = Event()

        def start(self):
            self.nosig = time.time()
            while gpio.input(self.echo) == 1:
                self.sig = time.time()
            self.done.set()
            gpio.remove_event_detect(self.echo)
        
        def arm(self):
            self.done.clear()
            gpio.add_event_detect(self.echo, gpio.RISING, callback=lambda x: self.start())
        
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
        gpio.output(self.trg, False)
        time.sleep(0.00001)

        gpio.output(self.trg, True)
        time.sleep(0.00001)
        gpio.output(self.trg, False)
        
    def run(self):
        
        self.arm()
        self.trigger()
        return self.measure()
    
    # return self._distance

class Tester:

    def __init__(self):

        self.pins = [7,11,12,13,15,16,18,21,22,23,24,26,29,31,32,33,35,36,37,38,40]
        
        gpio.setmode(gpio.BOARD)
        for pin in self.pins:
            ## print pin
            gpio.setup(pin,gpio.OUT)

    def on(self):
        for pin in self.pins:
            print("setting pin", pin)
            gpio.output(pin,gpio.HIGH)
            raw_input("")
            gpio.output(pin,gpio.LOW)

    def off(self):
        for pin in self.pins:
            gpio.output(pin,gpio.LOW)
            
    

if __name__ == "__main__":
    ## global motors
    ## motors = Motors()
    
    ## motors.forward()
    ## motors.turn_left()
    ## motors.forward()
    ## motors.turn_left()
    ## motors.forward()
    ## motors.turn_left()
    ## motors.forward()
    ## 
    ## motors.reverse()
    ## motors.turn_right()
    ## motors.reverse()
    ## motors.turn_right()
    ## motors.reverse()
    ## motors.turn_right()
    ## motors.reverse()
    
    
    ## motors = Motors(29,31,33,37,tsleep=5) ## 
    ## motors.send(gpio.HIGH,gpio.HIGH,gpio.HIGH,gpio.HIGH)
    ## time.sleep(4)
    ## motors.send(gpio.LOW,gpio.LOW,gpio.LOW,gpio.LOW)
    ## time.sleep(1)
    ## # gpio.cleanup()
    ## 
    ## print "fwd"
    ## 
    ## time.sleep(2)
    ## print "rev"
    ## motors.reverse()
    ## time.sleep(2)
    ### print "left"
    ### motors.turn_left()
    ### time.sleep(1)
    ### print "right"
    ### motors.turn_right()
    ### time.sleep(1)
    # motors.send(gpio.HIGH,gpio.HIGH,gpio.HIGH,gpio.HIGH)
    ## time.sleep(2)
    ## motors.send(gpio.LOW,gpio.LOW,gpio.LOW,gpio.LOW)
    ## time.sleep(1)
    ## 
    ## motors.stop()

    ### global sensor
    ### ## sensor = Sensor(16,18)
    # sensors = Sensors()
     
    ## for i in range(5):
    ##      print(sensors.run())
    ##      time.sleep(1)
         
     

    
    tester = Tester()
    tester.on()
    raw_input()
    gpio.cleanup()

    
