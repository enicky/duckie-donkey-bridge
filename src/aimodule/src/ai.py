#!/usr/bin/env python

import rospy
import torch
from sensor_msgs.msg import CompressedImage
import numpy as np
import td3
import cv2

VERBOSE = True


class AiDriver:
    def __init__(self):
        print("Init of AI Driver")
        rospy.loginfo('CTOR Donkey Driver')
        # all_param_names = rospy.get_param_names()
        # print("all found param names : ", all_param_names)
        vehicle_name = rospy.get_param('/%s/veh' % rospy.get_name())
        print("vehicle ame : ", vehicle_name)
        subscriber_camera_topic = "/%s/camera_node/image/compressed" % vehicle_name

        if VERBOSE:
            rospy.loginfo('[%s] Subscribing to topic "%s"'  % (rospy.get_name(), subscriber_camera_topic))

        self.sub_topic = rospy.Subscriber(subscriber_camera_topic, CompressedImage, self.on_camera_image, queue_size=1)

        state_dim = 3
        action_dim = 2
        max_action = 1.0
        max_turn_angle = 1.0

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        print("device found : ", self.device)
        self.policy = td3.TD3(state_dim, action_dim, max_action, max_turn_angle)
        print("policy made ... ")


    def on_camera_image(self, ros_data):
        if VERBOSE:
            rospy.loginfo("[%s] Receieved camera image of type : %s" % (rospy.get_name(), ros_data.format))
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        cv2.imshow('cv_img', image_np)

        cv2.waitKey(2)
        action = self.policy.select_action(np_arr)

        print("action : ", action)


    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." % (rospy.get_name()))


if __name__ == "__main__":
    rospy.init_node('duckie_ai_module', anonymous=False)
    node = AiDriver()
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
