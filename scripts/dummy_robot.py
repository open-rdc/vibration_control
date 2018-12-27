#!/usr/bin/env python
from __future__ import print_function
import roslib
import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import threading
import readchar
import copy

mobile_base_width = 0.4
mobile_base_height = 0.3
wire_length = 1.2

fig, ax = plt.subplots(1, 1)
mobile_base_rect = patches.Rectangle(xy = (-mobile_base_width / 2, 0), width = mobile_base_width, height = mobile_base_height, fc = 'm')
ax.add_patch(mobile_base_rect)
wire_line, = ax.plot([0, 0], [mobile_base_height, mobile_base_height + wire_length], lw=2)
weight_rect = patches.Rectangle(xy = (-0.1 / 2, mobile_base_height + wire_length), width = 0.1, height = 0.1, fc = 'r')
ax.add_patch(weight_rect)
plt.axis([-5, 5, 0, 2])

def draw_graphics(self):
    mobile_base_rect.set_xy((self.robot_x - mobile_base_width / 2, 0))
    wire_line.set_data([self.robot_x, self.wire_top_pos], [mobile_base_height, mobile_base_height + wire_length])
    weight_rect.set_xy((self.weight_pos - 0.1/2, mobile_base_height + wire_length))
    plt.pause(0.01)

class dummy_robot:
    DURATION = 0.01
    speed_up_factor = 1
    def __init__(self):
        rospy.init_node('dummy_robot', anonymous=True)
        self.joint_state_pub = rospy.Publisher('/manipulator/joint_states', JointState, queue_size=1)
        self.action_sub = rospy.Subscriber('/icart_mini/cmd_vel', Twist, self.callback_action)
        self.timer = rospy.Timer(rospy.Duration(dummy_robot.DURATION), self.callback_joint_state_timer)
        self.robot_vel_x = 0.3
        self.robot_x = 0
        self.wire_top_pos = 0
        self.wire_top_vel = 0
        self.weight_pos = 0
        self.weight_vel = 0
        self.wire_math = 0.5
        self.wire_spring_coef = 20.0
        self.wire_dumping = 0.1
        self.weight_math = 0.5
        self.state = JointState()
        self.state.position = [0.0, 0.0, 0.0]
        self.state.velocity = [0.0, 0.0, 0.0]
        self.robot_turn_counter = 0
        self.robot_turn_period = 5
        self.weight_moving_range = 0.1 # 0.0: disable control

    def callback_action(self, data):
        self.weight_vel = data.linear.x

    def callback_joint_state_timer(self, data):
        self.robot_x += self.robot_vel_x * dummy_robot.DURATION
        self.robot_turn_counter += dummy_robot.DURATION
        if self.robot_turn_counter >= self.robot_turn_period:
            self.robot_turn_counter = 0;
            self.robot_vel_x *= -1

        target_force = 3 * (self.robot_vel_x - self.wire_top_vel)
        target_weight_acc = - target_force / self.weight_math
        self.weight_vel += target_weight_acc * dummy_robot.DURATION
        self.weight_pos += self.weight_vel * dummy_robot.DURATION + target_weight_acc * dummy_robot.DURATION * dummy_robot.DURATION / 2
        if abs(self.weight_pos - self.wire_top_pos) >= self.weight_moving_range:
            target_force = 0
            self.weight_pos = self.wire_top_pos + self.weight_moving_range * np.sign(self.weight_pos - self.wire_top_pos)
            self.weight_vel = self.wire_top_vel

        wire_acc = (target_force + self.wire_spring_coef * (self.robot_x - self.wire_top_pos) + self.wire_dumping * (self.robot_vel_x - self.wire_top_vel)) / (self.wire_math + self.weight_math)
        self.wire_top_vel += wire_acc * dummy_robot.DURATION
        self.wire_top_pos += self.wire_top_vel * dummy_robot.DURATION + wire_acc * dummy_robot.DURATION * dummy_robot.DURATION / 2

        self.joint_state_pub.publish(self.state)

    def getkey(self):
        while True:
            self.key = readchar.readkey()

if __name__ == '__main__':
    dr = dummy_robot()
    r = rospy.Rate(1/dr.DURATION)
    while not rospy.is_shutdown():
        draw_graphics(dr)
        r.sleep()
