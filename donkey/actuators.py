# Simple car movement using the PCA9685 PWM servo/LED controller library.
#
# Attribution: hacked from sample code from Tony DiCola

import time
import sys

# Import the PCA9685 module.

# Uncomment to enable debug output.
#import logging
#logging.basicConfig(level=logging.DEBUG)


def map_range(x, X_min, X_max, Y_min, Y_max):
    X_range = X_max - X_min
    Y_range = Y_max - Y_min
    XY_ratio = X_range/Y_range

    y = ((x-X_min) / XY_ratio + Y_min) // 1

    return int(y)

class Dummy_Controller:

    def __init__(self, channel, frequency):
        pass

class PCA9685_Controller:
    # Init with 60hz frequency by default, good for servos.
    def __init__(self, channel, frequency=50):
        import Adafruit_PCA9685
        # Initialise the PCA9685 using the default address (0x40).
        self.pwm = Adafruit_PCA9685.PCA9685()

        self.pwm.set_pwm_freq(frequency)
        self.channel = channel

    def set_pulse(self, pulse):
        self.pwm.set_pwm(self.channel, 0, pulse)

class PWMSteeringActuator:
    #max angle wheels can turn
    LEFT_ANGLE = -45
    RIGHT_ANGLE = 45

    def __init__(self, controller=None,
                       left_pulse=290,
                       right_pulse=490):

        self.controller = controller
        self.left_pulse = left_pulse
        self.right_pulse = right_pulse

    def update(self, angle):
        #map absolute angle to angle that vehicle can implement.
        pulse = map_range(angle,
                          self.LEFT_ANGLE, self.RIGHT_ANGLE,
                          self.left_pulse, self.right_pulse)

        self.controller.set_pulse(pulse)


class PWMThrottleActuator:

    MIN_THROTTLE = -100
    MAX_THROTTLE =  100

    def __init__(self, controller=None,
                       max_pulse=300,
                       min_pulse=490,
                       zero_pulse=350):

        #super().__init__(channel, frequency)
        self.controller = controller
        self.max_pulse = max_pulse
        self.min_pulse = min_pulse
        self.zero_pulse = zero_pulse
        self.calibrate()


    def calibrate(self):
        #Calibrate ESC (TODO: THIS DOES NOT WORK YET)
        print('center: %s' % self.zero_pulse)
        self.controller.set_pulse(self.zero_pulse)  #Set Max Throttle
        time.sleep(1)


    def update(self, throttle):
        print('throttle update: %s' %throttle)
        if throttle > 0:
            pulse = map_range(throttle,
                              0, self.MAX_THROTTLE,
                              self.zero_pulse, self.max_pulse)
        else:
            pulse = map_range(throttle,
                              self.MIN_THROTTLE, 0,
                              self.min_pulse, self.zero_pulse)

        print('pulse: %s' % pulse)
        sys.stdout.flush()
        self.controller.set_pulse(pulse)
        return '123'
