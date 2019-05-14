#!/usr/bin/env python

import rospy
import time
import RPi.GPIO as GPIO
from datetime import datetime
import iothub_client
from iothub_client import IoTHubClient, IoTHubTransportProvider
from iothub_client import IoTHubMessage
import json


class ColorSensorDriver:
    def __init__(self):
        rospy.loginfo("[%s] Init Color Sensor" % rospy.get_name())
        self.signal = 25
        self.s2 = 23
        self.s3 = 24
        self.NUM_CYCLES = 10
        self.previous_timestamp = 0

        self.CONNECTION_STRING = self.setup_param("~connection_string", "DeviceId=falcon-ai;SharedAccessKey=JpY0FhGOivv3NfX09c/HlKjdIKKeOdseqtq2LrhyVdI=")
        self.PROTOCOL = IoTHubTransportProvider.MQTT

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.signal, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.s2, GPIO.OUT)

        GPIO.setup(self.s3, GPIO.OUT)
        self.iothub_client = self.iothub_client_init()

    def setup_param(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  # Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))

        return value

    def iothub_client_init(self):
        client = IoTHubClient(self.CONNECTION_STRING, self.PROTOCOL)
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
            data_object = self.create_json(now, self.previous_timestamp)
            message = IoTHubMessage(data_object)
            self.iothub_client.send_event_async(message)
            rospy.loginfo("[%s] Sent message to IoTHub " % (rospy.get_name()))
            self.previous_timestamp = now

    def create_json(self, now, previous_timestamp):
        """
        Create json out of timestamps => only usable by AI car
        :param now:
        :param previous_timestamp:
        :return:
        """
        x = {
            "track": "falcon",
            "car": "AI",
            "now": now,
            "previous": previous_timestamp
        }
        y = json.dump(x)
        rospy.logdebug("[%s] created json : %s" % (rospy.get_name(), y))
        return y


if __name__ == '__main__':
    rospy.init_node('sens', anonymous=False)
    node = ColorSensorDriver()
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
