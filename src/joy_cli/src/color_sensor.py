#!/usr/bin/env python

import rospy
import time
import RPi.GPIO as GPIO
from datetime import datetime


class ColorSensorDriver:
    def __init__(self):
        rospy.loginfo("[%s] Init Color Sensor" % rospy.get_name())
        self.signal = 25
        self.s2 = 23
        self.s3 = 24
        self.NUM_CYCLES = 10

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.signal, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.s2, GPIO.OUT)

        GPIO.setup(self.s3, GPIO.OUT)



    def on_shutdown(self):
        rospy.logdebug("[%s] Shutdown ColorSensor Driver" % rospy.get_name())

    def sense_color(self):
        GPIO.output(self.s2, GPIO.LOW)
        GPIO.output(self.s3, GPIO.LOW)
        time.sleep(0.3)
        start = time.time()
        for impulse_count in range(self.NUM_CYCLES):
            GPIO.wait_for_edge(self.signal, GPIO.FALLING)
        duration = time.time() - start

        red = self.NUM_CYCLES / duration

        print("Found red :  %s " % red)
        if red > 12000:
            now = datetime.timestamp()
            rospy.loginfo("[%s] %s => Found color red: %s" % (rospy.get_name(), str(now), red))



if __name__ == '__main__':
    rospy.init_node('sens', anonymous=False)
    node = ColorSensorDriver()
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
