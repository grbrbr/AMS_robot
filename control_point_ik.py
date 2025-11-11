#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
control_point_ik.py

A ROS node implementing a simple proportional controller to drive a differential
drive robot to a goal point. It computes desired linear and angular velocities
(v, omega) from pose error, then maps them to wheel angular velocities using
inverse kinematics.

Publishes:
 - "cmd_vel" (geometry_msgs/Twist) for compatibility
 - "wheel_left_cmd" and "wheel_right_cmd" (std_msgs/Float32) wheel angular velocities [rad/s]

Parameters (ROS params or change constants below):
 - goal_x, goal_y, goal_phi
 - wheel_radius, wheel_base
 - Kp_linear, Kp_angular, Kp_final_theta
 - max_linear, max_angular, max_wheel
 - stop_distance

This node intentionally doesn't assume any specific motor driver topic names;
adapt the published topics to your platform if needed.
"""

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import ams
from math import atan2

# --- Parameters (tweak as needed or expose as ROS params) ---
GOAL_X = rospy.get_param('~goal_x', 0.1)
GOAL_Y = rospy.get_param('~goal_y', 0.0)
GOAL_PHI = rospy.get_param('~goal_phi', 0.0)

WHEEL_RADIUS = rospy.get_param('~wheel_radius', 0.05)   # meters
WHEEL_BASE = rospy.get_param('~wheel_base', 0.30)      # distance between wheels (meters)

Kp_linear = rospy.get_param('~Kp_linear', 1.2)
Kp_angular = rospy.get_param('~Kp_angular', 2.0)
Kp_final_theta = rospy.get_param('~Kp_final_theta', 2.0)

MAX_LINEAR = rospy.get_param('~max_linear', 0.3)       # m/s
MAX_ANGULAR = rospy.get_param('~max_angular', 1.0)     # rad/s
MAX_WHEEL = rospy.get_param('~max_wheel', 10.0)        # rad/s

STOP_DISTANCE = rospy.get_param('~stop_distance', 0.03)  # meters

# Topics
CMD_VEL_TOPIC = rospy.get_param('~cmd_vel_topic', 'cmd_vel')
WHEEL_LEFT_TOPIC = rospy.get_param('~wheel_left_topic', 'wheel_left_cmd')
WHEEL_RIGHT_TOPIC = rospy.get_param('~wheel_right_topic', 'wheel_right_cmd')

# Internal state
pub_cmd_vel = None
pub_wl = None
pub_wr = None


def chassis_to_wheels(v, omega, wheel_base, wheel_radius):
    """Map chassis linear velocity v (m/s) and angular velocity omega (rad/s)
    to left and right wheel angular velocities [rad/s].
    """
    # v = (wr + wl) * R / 2
    # omega = (wr - wl) * R / L
    # Solve for wl, wr
    wl = (v - (omega * wheel_base / 2.0)) / wheel_radius
    wr = (v + (omega * wheel_base / 2.0)) / wheel_radius
    return wl, wr


def limit(value, vmin, vmax):
    if value < vmin:
        return vmin
    if value > vmax:
        return vmax
    return value


def odom_cb(msg):
    global pub_cmd_vel, pub_wl, pub_wr

    x, y, phi = ams.msgToPose(msg.pose.pose)

    # Pose errors
    dx = GOAL_X - x
    dy = GOAL_Y - y
    dist = (dx*dx + dy*dy) ** 0.5

    angle_to_goal = atan2(dy, dx)
    # Heading error to face the goal
    heading_error = ams.wrapToPi(angle_to_goal - phi)

    # If close to the goal position, switch to final orientation control
    if dist < STOP_DISTANCE:
        # Position reached; rotate to desired final orientation
        theta_error = ams.wrapToPi(GOAL_PHI - phi)

        v = 0.0
        omega = Kp_final_theta * theta_error
        # limit omega
        omega = limit(omega, -MAX_ANGULAR, MAX_ANGULAR)

        if abs(theta_error) < 0.02:  # small threshold -> stop
            omega = 0.0

    else:
        # Drive towards the goal: simple proportional controller
        v = Kp_linear * dist
        omega = Kp_angular * heading_error

        # Limit
        v = limit(v, -MAX_LINEAR, MAX_LINEAR)
        omega = limit(omega, -MAX_ANGULAR, MAX_ANGULAR)

    # Compute wheel angular velocities (rad/s)
    wl, wr = chassis_to_wheels(v, omega, WHEEL_BASE, WHEEL_RADIUS)
    wl = limit(wl, -MAX_WHEEL, MAX_WHEEL)
    wr = limit(wr, -MAX_WHEEL, MAX_WHEEL)

    # Publish a Twist for compatibility with navigation stacks
    twist = Twist()
    twist.linear.x = v
    twist.angular.z = omega
    pub_cmd_vel.publish(twist)

    # Publish wheel commands (rad/s)
    msg_wl = Float32(); msg_wl.data = wl
    msg_wr = Float32(); msg_wr.data = wr
    pub_wl.publish(msg_wl)
    pub_wr.publish(msg_wr)


def main():
    global pub_cmd_vel, pub_wl, pub_wr

    rospy.init_node('control_point_ik')

    # Publishers
    pub_cmd_vel = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=1)
    pub_wl = rospy.Publisher(WHEEL_LEFT_TOPIC, Float32, queue_size=1)
    pub_wr = rospy.Publisher(WHEEL_RIGHT_TOPIC, Float32, queue_size=1)

    # Subscriber
    sub = rospy.Subscriber('odom', Odometry, odom_cb, queue_size=1)

    rospy.loginfo("control_point_ik node started. Goal: (%.3f, %.3f, %.3f)", GOAL_X, GOAL_Y, GOAL_PHI)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
