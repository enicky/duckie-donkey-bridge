import time

class PCA9685:
    '''
       PWM motor controler using PCA9685 boards.
       This is used for most RC Cars
       '''

    def __init__(self, channel, address=0x40, frequency=60, busnum=None):
        import Adafruit_PCA9685
        # Initialise the PCA9685 using the default address (0x40).
        if busnum is not None:
            from Adafruit_GPIO import I2C
            # replace the get_bus function with our own
            def get_bus():
                return busnum

            I2C.get_default_bus = get_bus
        self.pwm = Adafruit_PCA9685.PCA9685(address=address)
        self.pwm.set_pwm_freq(frequency)
        self.channel = channel

    def set_pulse(self, pulse):
        self.pwm.set_pwm(self.channel, 0, pulse)

    def run(self, pulse):
        self.set_pulse(pulse)


class PWMSteering:
    """
        Wrapper over a PWM motor cotnroller to convert angles to PWM pulses.
        """
    LEFT_ANGLE = -1
    RIGHT_ANGLE = 1

    def __init__(self, controller=None,
                 left_pulse=290,
                 right_pulse=490):
        self.controller = controller
        self.left_pulse = left_pulse
        self.right_pulse = right_pulse

    def run(self, angle):
        # map absolute angle to angle that vehicle can implement.
        pulse = self.map_range(angle,
                                   self.LEFT_ANGLE, self.RIGHT_ANGLE,
                                   self.left_pulse, self.right_pulse)

        self.controller.set_pulse(pulse)

    def shutdown(self):
        self.run(0)  # set steering straight

    def map_range(self, x, X_min, X_max, Y_min, Y_max):
        """
        Linear mapping between two ranges of values
        """
        X_range = X_max * 1.0 - X_min * 1.0
        Y_range = Y_max * 1.0 - Y_min * 1.0
        XY_ratio = X_range / Y_range

        y = ((x - X_min) / XY_ratio + Y_min) // 1

        return int(y)


class PWMThrottle:
    """
        Wrapper over a PWM motor cotnroller to convert -1 to 1 throttle
        values to PWM pulses.
        """
    MIN_THROTTLE = -1
    MAX_THROTTLE = 1

    def __init__(self, controller=None,
                 max_pulse=300,
                 min_pulse=490,
                 zero_pulse=350):

        self.controller = controller
        self.max_pulse = max_pulse
        self.min_pulse = min_pulse
        self.zero_pulse = zero_pulse

        # send zero pulse to calibrate ESC
        print("Init ESC")
        self.controller.set_pulse(self.max_pulse)
        time.sleep(0.01)
        self.controller.set_pulse(self.min_pulse)
        time.sleep(0.01)
        self.controller.set_pulse(self.zero_pulse)
        time.sleep(1)
        print("Finished init of ESC")

    def run(self, throttle):
        if throttle > 0:
            pulse = self.map_range(throttle,
                                       0, self.MAX_THROTTLE,
                                       self.zero_pulse, self.max_pulse)
        else:
            pulse = self.map_range(throttle,
                                       self.MIN_THROTTLE, 0,
                                       self.min_pulse, self.zero_pulse)

        self.controller.set_pulse(pulse)

    def shutdown(self):
        self.run(0)  # stop vehicle

    def map_range(self, x, X_min, X_max, Y_min, Y_max):
        """
        Linear mapping between two ranges of values
        """
        X_range = X_max * 1.0 - X_min * 1.0
        Y_range = Y_max * 1.0 - Y_min * 1.0
        XY_ratio = X_range / Y_range

        y = ((x - X_min) / XY_ratio + Y_min) // 1

        return int(y)
