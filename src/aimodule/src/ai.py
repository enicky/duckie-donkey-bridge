#!/usr/bin/env python
from collections import deque

import rospy
import torch
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import ActionCmd
import numpy as np
import td3
import cv2
from scipy.misc import imresize
from skimage import color

VERBOSE = False
SHOW_VIDEO = False

class AiDriver:
    def __init__(self):
        vehicle_name = rospy.get_param('/%s/veh' % rospy.get_name())
        subscriber_camera_topic = "/%s/camera_node/image/compressed" % vehicle_name
        publisher_action_topic = "/%s/action" % vehicle_name

        if VERBOSE:
            rospy.loginfo('[%s] Subscribing to topic "%s"' % (rospy.get_name(), subscriber_camera_topic))

        self.sub_topic = rospy.Subscriber(subscriber_camera_topic, CompressedImage, self.on_camera_image, queue_size=1)
        self.pub_action = rospy.Publisher(publisher_action_topic, ActionCmd, queue_size=1)

        state_dim = 3
        action_dim = 2
        max_action = 1.0
        max_turn_angle = 1.0

        self.obs_hi = 255.0
        self.obs_lo = 0.0

        self.policy = td3.TD3(state_dim, action_dim, max_action, max_turn_angle)
        self.target_reshape_size = (120, 160, 3)

        self.max_framestack = 3
        self.frames = deque([], maxlen=self.max_framestack)

    def preprocess_image(self, np_arr):
        if VERBOSE:
            rospy.loginfo("[%s] Preprocessing image for pytorch stuff" % rospy.get_name())
        ob = self.resize_wrapper(np_arr)
        resized = ob
        ob = self.grayscale_wrapper(ob)
        grey = ob
        ob = self.normalize_wrapper(ob)
        ob = self.framestack_wrapper(ob, 3)

        ob = self.img_wrapper(ob)
        if SHOW_VIDEO:
            return ob, resized, grey
        return ob, None, None

    def on_camera_image(self, ros_data):
        if VERBOSE:
            rospy.loginfo("[%s] Receieved camera image of type : %s" % (rospy.get_name(), ros_data.format))
        np_arr = np.fromstring(ros_data.data, np.uint8)

        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        processed_image, resized, greimage = self.preprocess_image(image_np)
        if SHOW_VIDEO:
            cv2.imshow('cv_img', greimage)
            cv2.imshow('resized', resized)
            cv2.waitKey(2)
        action = self.policy.select_action(processed_image)
        cmd = ActionCmd()
        cmd.vel = action[0]
        cmd.angle = action[1]

        self.send_action(cmd)
        if VERBOSE:
            print("action : ", action)


    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." % (rospy.get_name()))

    def resize_wrapper(self, observation):
        reresized = imresize(observation, self.target_reshape_size)
        return reresized

    def grayscale_wrapper(self, observation):
        obs = color.rgb2gray(np.asarray(observation, dtype=np.uint8)).reshape(observation.shape[0],
                                                                              observation.shape[1], 1)
        return obs

    def normalize_wrapper(self, ob):
        return (ob - self.obs_lo) / (self.obs_hi - self.obs_lo)

    def framestack_wrapper(self, ob, param):
        self.frames.append(ob)
        assert len(self.frames) == self.max_framestack
        return LazyFrames(list(self.frames))

    def img_wrapper(self, ob):
        return ob.__array__().transpose(2, 0, 1)

    def send_action(self, cmd):
        self.pub_action.publish(cmd)


class LazyFrames(object):
    def __init__(self, frames):
        """This object ensures that common frames between the observations are only stored once.
        It exists purely to optimize memory usage which can be huge for DQN's 1M frames replay
        buffers.
        This object should only be converted to numpy array before being passed to the model.
        You'd not believe how complex the previous solution was."""
        self._frames = frames
        self._out = None

    def _force(self):
        if self._out is None:
            self._out = np.concatenate(self._frames, axis=-1)
            self._frames = None
        return self._out

    def __array__(self, dtype=None):
        out = self._force()
        if dtype is not None:
            out = out.astype(dtype)
        return out

    def __len__(self):
        return len(self._force())

    def __getitem__(self, i):
        return self._force()[..., i]


if __name__ == "__main__":
    rospy.init_node('duckie_ai_module', anonymous=False)
    rospy.Rate(10)
    node = AiDriver()
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
