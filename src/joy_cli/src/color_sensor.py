#!/usr/bin/env python

import rospy
import time
import RPi.GPIO as GPIO
from datetime import datetime
import iothub_client
from iothub_client import IoTHubClient, IoTHubTransportProvider
from iothub_client import IoTHubMessage

PROTOCOL = IoTHubTransportProvider.MQTT

# String containing Hostname, Device Id & Device Key in the format:
# "HostName=<host_name>;DeviceId=<device_id>;SharedAccessKey=<device_key>"
CONNECTION_STRING = "[Device Connection String]"


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
        self.iothub_client = self.iothub_client_init()


    def iothub_client_init(self):
        client = IoTHubClient(CONNECTION_STRING, PROTOCOL)
        return client

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
            message = IoTHubMessage('{"data":true}')
            self.iothub_client.send_event_async(message)
            rospy.loginfo("[%s] Sent message to IoTHub " % (rospy.get_name()))


if __name__ == '__main__':
    rospy.init_node('sens', anonymous=False)
    node = ColorSensorDriver()
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
