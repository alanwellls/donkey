"""
Rotary Encoder
"""

import time

class RotaryEncoder():
    def __init__(self, mm_per_tick=0.5769, pin=27):
        import RPi.GPIO as GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(pin, GPIO.BOTH, callback=self.isr)

        # initialize the odometer values
        self.m_per_tick = mm_per_tick / 1000.0
        self.meters = 0
        self.last_time = time.time()
        self.meters_per_second = 0
        self.counter = 0
        self.top_speed = 0
        self.on = True
    
    def isr(self, channel):
        self.counter += 1
        
    def update(self):
        # keep looping infinitely until the thread is stopped
        while(self.on):
                
            #save the ticks and reset the counter
            ticks = self.counter
            self.counter = 0
            
            #save off the last time interval and reset the timer
            start_time = self.last_time
            end_time = time.time()
            self.last_time = end_time
            
            #calculate elapsed time and distance traveled
            seconds = end_time - start_time
            distance = ticks * self.m_per_tick
            velocity = distance / seconds
            
            #update the odometer values
            self.meters += distance
            self.meters_per_second = velocity

            if velocity > self.top_speed:
                self.top_speed = velocity

            #console output for debugging
            if(self.counter > 0):
                print('distance (m):', round(self.meters, 4))
                print('velocity (m/s):', round(self.meters_per_second, 4))

    def run_threaded(self):
        return self.meters, self.meters_per_second

    def shutdown(self):
        # indicate that the thread should be stopped
        self.on = False
        print('stopping Rotary Encoder')
        print('top speed (m/s):', self.top_speed)
        time.sleep(.5)
        
        import RPi.GPIO as GPIO
        GPIO.cleanup() 
