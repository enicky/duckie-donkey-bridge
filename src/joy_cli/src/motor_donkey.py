#!/usr/bin/env python
# license removed for brevity

import rospy
from duckietown_msgs.msg import WheelsCmdStamped, BoolStamped, Twist2DStamped
from sensor_msgs.msg import Joy
from pca_driver import PCA9685, PWMSteering, PWMThrottle
import os


class DonkeyCarDriver:
    def __init__(self):
        print("ctor")
        rospy.loginfo('CTOR Donkey Driver')
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " % self.node_name)

        self.estop = False

        # Parameters for maximal turning radius
        self.use_rad_lim = self.setupParam("~use_rad_lim", False)
        self.min_rad = self.setupParam("~min_rad", 0.08)

        self.wheel_distance = self.setupParam("~wheel_distance", 0.103)

        self.pca9685_i2c_address = self.setupParam('~pca9685_i2c_address', 0x40)
        self.pca9685_i2c_busnum = None

        self.throttle_channel = self.setupParam("~throttle_channel", 0)
        self.throttle_forward_pwm = self.setupParam('~throttle_forward_pwm', 500)
        self.throttle_stopped_pwm = self.setupParam('~throttle_stopped_pwm', 370)
        self.throttle_reverse_pwm = self.setupParam('~throttle_reverse_pwm', 220)

        self.steering_channel = self.setupParam("~steering_channel", 1)
        self.steering_left_pwm = self.setupParam("~steering_left_pwm", 460)
        self.steering_right_pwm = self.setupParam("~steering_right_pwm", 290)

        self.veh_name = os.environ['VEHICLE_NAME']

        # Setup subscribers
        self.control_constant = 1.0

        subscriber_topic_name = "/" + str(self.veh_name) + '/joy'
        subscriber_wheel_cmd_name = "/" + str(self.veh_name) + '/wheels_driver_node/wheels_cmd'

        rospy.loginfo('[%s] Subscribing to topic : "%s"' % (self.node_name, subscriber_topic_name))

        self.sub_topic = rospy.Subscriber(subscriber_topic_name, Joy, self.on_wheels_cmd)
        self.sub_topic_wheels_cmd = rospy.Subscriber(subscriber_wheel_cmd_name, WheelsCmdStamped, self.on_wheels_cmd_cmd)
        rospy.loginfo("done subscribing ... ")
        self.sub_e_stop = rospy.Subscriber("~emergency_stop", BoolStamped, self.on_e_stop, queue_size=1)
        self.sub_wheels_command = rospy.Subscriber('~wheels_cmd', WheelsCmdStamped, self.on_wheels_internal)

        self.params_update = rospy.Timer(rospy.Duration.from_sec(1.0), self.update_params)

        self.throttle_controller = PCA9685(self.throttle_channel, address=self.pca9685_i2c_address,
                                           busnum=self.pca9685_i2c_busnum)
        self.throttle_driver = PWMThrottle(controller=self.throttle_controller,
                                           max_pulse=self.throttle_forward_pwm,
                                           zero_pulse=self.throttle_stopped_pwm,
                                           min_pulse=self.throttle_reverse_pwm)

        self.steering_controller = PCA9685(self.steering_channel, address=self.pca9685_i2c_address,
                                           busnum=self.pca9685_i2c_busnum)
        self.steering_driver = PWMSteering(self.steering_controller, left_pulse=self.steering_left_pwm,
                                           right_pulse=self.steering_right_pwm)

    def setupParam(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  # Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))

        return value

    def update_params(self, event):
        self.use_rad_lim = rospy.get_param("~use_rad_lim")
        self.min_rad = rospy.get_param("~min_rad")
        self.wheel_distance = rospy.get_param("~wheel_distance")

    def on_shutdown(self):
        # self.driver.setWheelsSpeed(left=0.0, right=0.0)
        self.throttle_driver.run(0.0)
        self.steering_driver.run(0.0)

        rospy.loginfo("[%s] Shutting down." % (rospy.get_name()))

    def on_e_stop(self, data):
        self.estop = not self.estop

        if self.estop:
            rospy.loginfo("[%s] Emergency Stop Activated")
        else:
            rospy.loginfo("[%s] Emergency Stop Released")

    def on_wheels_cmd(self, msg):
        rospy.loginfo('got wheels_cmd')
        print("got wheelscmd : ", msg)

        if self.estop:
            rospy.loginfo('Emergency STOP !!!')
            self.throttle_driver.run(0.0)
            return

    def on_wheels_cmd_cmd(self, msg):
        print("Processed wheelscmd", msg)

    def on_wheels_internal(self, msg):
        rospy.loginfo('got wheels internal message : ')
        print("got wheelsy ... ", msg)

    def on_car_cmd(self, msg):
        rospy.loginfo("got car cmd messge")
        print("got car cmd message : ", msg)


if __name__ == '__main__':
    rospy.init_node('donkey_car_driver', anonymous=False)
    node = DonkeyCarDriver()
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()