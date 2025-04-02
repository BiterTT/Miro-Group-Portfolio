# create node
import rospy
rospy.init_node("client_minimal", anonymous=True)

################################################################

# to use in MIROcode, copy everything below this line into the
# MIROcode Python editor.
#
# vvvvvv vvvvvv vvvvvv vvvvvv

################################################################

import os
import sys
import time
import numpy as np
import math

import miro2 as miro
import geometry_msgs
from geometry_msgs import *
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import UInt8, UInt16, UInt32, Float32MultiArray, UInt16MultiArray, UInt32MultiArray
from sensor_msgs.msg import Range

import rospy
from sensor_msgs.msg import JointState
from vosk import Model, KaldiRecognizer
import sounddevice as sd
import json
import threading
################################################################

class MiroSensors:
    def __init__(self):
        self.sonar_distance = 0
        self.cliff_left = -1
        self.cliff_right = -1
        self.cliff_flag = None

        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        topic = topic_base_name + "/sensors/package"
        rospy.Subscriber(topic, miro.msg.sensors_package, self.sonar_callback,
                         queue_size=1, tcp_nodelay=True)
        topic = topic_base_name + "/sensors/cliff"
        rospy.Subscriber(topic, Float32MultiArray, self.cliff_callback,
                         queue_size=1, tcp_nodelay=True)

        rospy.loginfo(f"[MiroSensors] Subscribing to {topic}")

    def sonar_callback(self, msg):
        self.sonar_distance = msg.sonar.range

    def cliff_callback(self, msg):

        self.cliff_left = msg.data[0]
        self.cliff_right = msg.data[1]
        print(self.cliff_right)
        print(self.cliff_left)

    def get_sonar_distance(self):

        return self.sonar_distance

    def detect_cliff(self, threshold=0.5):
        if (self.cliff_left != -1) | (self.cliff_right != -1):
            if (self.cliff_left < threshold) & (self.cliff_right < threshold):
                self.cliff_flag = "inverse"
            elif (self.cliff_left < threshold) & (self.cliff_right > threshold):
                self.cliff_flag = "right"
            elif (self.cliff_left > threshold) & (self.cliff_right < threshold):
                self.cliff_flag = "left"
            else:
                self.cliff_flag = "cliff_safe"

class controller:
    # only need to copy the content of the function into class controller: of final project .py
    def loop(self):
        while self.t_now < 1000.0 and not rospy.core.is_shutdown():
            if self.debug_avoidance:
                self.update_avoidance_state()
                self.avoidance_motion()

    def update_avoidance_state(self):
        safe_dist = 0.25
        # clear states
        # self.avoidance_turn_left = False
        # self.avoidance_turn_right = False
        # self.avoidance_turn_back = False
        # self.avoidance_inverse = False

        self.sensors.detect_cliff()
        self.cliff_flag = self.sensors.cliff_flag
        dist = self.sensors.get_sonar_distance()
        # if dist is None:
        #     return

        if self.cliff_flag == "right":
            self.avoidance_turn_right = True
            print(self.cliff_flag)
        elif self.cliff_flag == "left":
            self.avoidance_turn_left = True
            print(self.cliff_flag)
        elif self.cliff_flag == "inverse":
            self.avoidance_inverse = True
            print(self.cliff_flag)
        # print(self.cliff_flag)
        # elif self.cliff_flag == "cliff_safe":
        if ((dist <= safe_dist) & (self.dist)):
            print("sonar")
            self.Wheel_Move_Straight_Forward(self.msg_wheels, "stop", -0.0)
            self.dist = False
            self.avoidance_turn_back = True

        self.avoidance_time = 0

    def __init__(self, args):

        # avoidance func

        self.debug_avoidance = True

        self.sensors = MiroSensors()
        self.avoidance_turn_left = False
        self.avoidance_turn_right = False
        self.avoidance_turn_back = False
        self.avoidance_inverse = False

        self.avoidance_time = 0
        self.avoidance_duration = 60

        self.dist = True

        self.cliff_flag = "cliff_safe"
