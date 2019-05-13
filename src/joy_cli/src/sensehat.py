#!/usr/bin/env python

from sense_hat import SenseHat, SenseStick, ACTION_RELEASED
from duckietown_msgs.msg import AiModeSelectionCmd
import rospy

red = (255, 0, 0)

class SenseHatDriver:
    def __init__(self):
        rospy.loginfo("[%s] Start CTOR SenshatDriver" % rospy.get_name())

        self.sense = SenseHat()
        self.sense.clear()
        self.stick = SenseStick()

        # Parameters
        self.sense_hat_param_enable = self.setup_param("~sense_hat_param_enable", True)

        self.veh_name = rospy.get_param('/%s/veh' % rospy.get_name())

        self.stick.direction_down = self.direction_down
        self.stick.direction_middle = self.direction_middle

        # modes
        self.current_mode = self.target_mode = ''
        self.current_led_status = self.target_led_status = 0

        # publish / subscriber topics
        self.publish_topic = '/%s/ai_mode_selection' % self.veh_name

        # publishers
        self.pub_mode = rospy.Publisher(self.publish_topic, AiModeSelectionCmd, queue_size=1)

        # parameters update
        self.params_update = rospy.Timer(rospy.Duration.from_sec(1.0), self.update_params)

    def setup_param(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  # Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (rospy.get_name(), param_name, value))

        return value

    def update_params(self, event):
        self.sense_hat_param_enable = rospy.get_param("~sense_hat_param_enable")

    def on_shutdown(self):
        self.sense.clear()

    def direction_middle(self, event):
        if event.action != ACTION_RELEASED:

            self.current_mode = self.target_mode
            self.current_led_status = self.target_led_status
            self.publish_ai_mode(self.current_led_status, self.current_mode)
            rospy.loginfo("[%s] published ai mode : %s %s" % (rospy.get_name(), str(self.current_led_status), str(self.current_mode)))
            self.sense.show_message("Mode %s" % str(self.current_mode), text_colour=red)
            self.sense.clear()
            # test

    def direction_down(self, event):
        if event.action != ACTION_RELEASED:
            if self.target_mode == 'user':
                self.target_mode = 'local_angle'
                self.target_led_status = 2
            elif self.target_mode == 'local_angle':
                self.target_mode = 'local'
                self.target_led_status = 3
            else:
                self.target_mode = 'user'
                self.target_led_status = 1

            rospy.loginfo("[%s] Target Mode : '%s', target led status : '%s'" % (rospy.get_name(), self.target_mode, str(self.target_led_status)))

    def publish_ai_mode(self, current_led_status, current_mode):
        message = AiModeSelectionCmd()
        message.mode = current_mode
        message.led_status = current_led_status
        rospy.loginfo("[%s] Start publishing message : %s" % (rospy.get_name(), message))
        self.pub_mode.publish(message)
        rospy.loginfo("[%s] Done publishing message" % rospy.get_name())


if __name__ == '__main__':
    rospy.init_node('senshat_driver', anonymous=False)
    node = SenseHatDriver()
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
