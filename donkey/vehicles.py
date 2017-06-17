'''
vehicles.py

Class to pull together all parts that operate the vehicle including,
sensors, actuators, pilots and remotes.
'''

import time

global ticks
ticks = 0

class BaseVehicle:
    def __init__(self,
                 drive_loop_delay = .5,
                 camera=None,
                 actuator_mixer=None,
                 pilot=None,
                 remote=None,
                 odometer=(False, 0.0)):

        self.drive_loop_delay = drive_loop_delay #how long to wait between loops

        #these need tobe updated when vehicle is defined
        self.camera = camera
        self.actuator_mixer = actuator_mixer
        self.pilot = pilot
        self.remote = remote
        self.odometer = odometer
        
        self.odometer_timestamps = []
        
        self.velocity = 0.0
        self.distance = 0.0
        
    def start(self):
        start_time = time.time()
        angle = 0.
        throttle = 0.
        
        if(self.odometer[0]):
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(23, GPIO.IN)
            GPIO.add_event_detect(23, GPIO.BOTH, callback=self.odometer_isr)


        #drive loop
        while True:
            now = time.time()
            start = now

            milliseconds = int( (now - start_time) * 1000)

            #get image array image from camera (threaded)
            img_arr = self.camera.capture_arr()

            angle, throttle, drive_mode = self.remote.decide_threaded(img_arr,
                                                 angle, 
                                                 throttle,
                                                 milliseconds)

            if drive_mode == 'local':
                angle, throttle = self.pilot.decide(img_arr)

            if drive_mode == 'local_angle':
                #only update angle from local pilot
                angle, _ = self.pilot.decide(img_arr)

            self.actuator_mixer.update(throttle, angle)
            

            global ticks
            
            if((self.odometer[0]) & (ticks > 0)):
                print("ticks", ticks)
                last_count = ticks
                ticks = 0
                            
                #increment the distance counter
                self.distance += last_count * self.odometer[1] 
                
                #calculate velocity
                self.velocity = (self.odometer[1] * last_count) / self.drive_loop_delay
                
            #print current car state
            end = time.time()
            lag = end - start
            print('\r CAR: angle: {:+04.2f}   throttle: {:+04.2f}   drive_mode: {}  lag: {:+04.2f}  velocity: {:+04.2f}'.format(angle, throttle, drive_mode, lag, self.velocity), end='')           
            
            time.sleep(self.drive_loop_delay)
            
    def odometer_isr(arg1, arg2):
        global ticks
        ticks += 1