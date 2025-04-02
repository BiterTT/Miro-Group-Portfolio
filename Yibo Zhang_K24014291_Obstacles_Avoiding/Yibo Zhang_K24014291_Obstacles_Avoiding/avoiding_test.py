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


import rospy
from sensor_msgs.msg import JointState
from vosk import Model, KaldiRecognizer
import sounddevice as sd
import json
import threading
################################################################

class MiroSensors:
    def __init__(self):
        self.sonar_distance = None
        self.cliff_left = None
        self.cliff_right = None

        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        topic = topic_base_name + "/sensors/package"
        rospy.Subscriber(topic, miro.msg.sensors_package, self._callback,
                         queue_size=1, tcp_nodelay=True)
        rospy.loginfo(f"[MiroSensors] Subscribing to {topic}")

    def _callback(self,msg):
        self.sonar_distance = msg.sonar.range
        self.cliff_left = msg.cliff.data[0]
        self.cliff_right = msg.cliff.data[1]

    def get_sonar_distance(self):
        return self.sonar_distance

    def detect_cliff(self, threshold=0.1, side="both"):
        if self.cliff_left is None or self.cliff_right is None:
            return False

        if side == "left":
            return self.cliff_left < threshold
        elif side == "right":
            return self.cliff_right < threshold
        elif side == "both":
            return self.cliff_left < self.cliff_right or self.cliff_right < threshold
        else:
            rospy.logwarn(f"[MiroSensors] detect_cliff() side={side} not supported")
            return False

class controller: #only need to copy the content of the function into class controller: of test4.0.py
    def loop(self):
        # loop
        while self.t_now < 1000.0 and not rospy.core.is_shutdown():
            if self.debug_avoidance:
                self.update_avoidance_state()
                self.avoidance_test_func()

            if self.debug_avoidance:
                current_time = time.time()
                if current_time - self.avoidance_start_time < self.avoidance_duration:
                    self.update_avoidance_state()
                    self.avoidance_test_func()
                    continue  # 跳过后续逻辑
                else:
                    rospy.loginfo("[Avoidance] 避障阶段结束，切换到其他功能")
                    self.debug_avoidance = False

    def update_avoidance_state(self):
        safe_dist = 0.25
        dist = self.sensors.get_sonar_distance()
        cliff_l = self.sensors.detect_cliff(side="left")
        cliff_r = self.sensors.detect_cliff(side="right")

        if dist is None:
            return

        # clear states
        self.avoidance_forward = False
        self.avoidance_turn_left = False
        self.avoidance_turn_right = False
        self.avoidance_turn_back = False

        # state adjust
        # if not cliff_l and not cliff_r and dist > safe_dist:
        #     self.avoidance_forward = True
        if cliff_l:
            self.avoidance_turn_right = True
        elif cliff_r:
            self.avoidance_turn_left = True
        elif dist <= safe_dist:
            self.avoidance_turn_back = True

        self.avoidance_time = 0  # 重置计时器

    def avoidance_test_func(self):
        if self.avoidance_forward:
            self.avoidance_time += 1
            if self.avoidance_time < self.avoidance_duration:
                self.Wheel_Move_Straight_Forward(self.msg_wheels, "move", 0.)
            else:
                self.avoidance_forward = False
                self.Wheel_Move_Straight_Forward(self.msg_wheels, "stop", 0.0)

        if self.avoidance_turn_left:
            self.avoidance_time += 1
            if self.avoidance_time < self.avoidance_duration:
                self.Spin(self.msg_spin, "spin_angles", 0.2)  # 左转90°
            else:
                self.avoidance_turn_left = False
                self.Spin(self.msg_spin, "stop", 0)

        elif self.avoidance_turn_right:
            self.avoidance_time += 1
            if self.avoidance_time < self.avoidance_duration:
                self.Spin(self.msg_spin, "spin_angles", -0.2)  # 右转90°
            else:
                self.avoidance_turn_right = False
                self.Spin(self.msg_spin, "stop", 0)

        elif self.avoidance_turn_back:
            self.avoidance_time += 1
            if self.avoidance_time < self.avoidance_duration:
                self.Spin(self.msg_spin, "spin_angles", -0.4)  # 掉头180°
            else:
                self.avoidance_turn_back = False
                self.Spin(self.msg_spin, "stop", 0)

    def __init__(self, args):

        # avoidance func
        self.debug_avoidance = True
        self.avoidance_start_time = time.time()
        self.avoidance_duration = 60

        self.sensors = MiroSensors()
        self.avoidance_forward = False
        self.avoidance_turn_left = False
        self.avoidance_turn_right = False
        self.avoidance_turn_back = False

        self.avoidance_time = 0
        self.avoidance_duration = 50  # should be adjusted
