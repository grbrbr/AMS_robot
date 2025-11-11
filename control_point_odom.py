#!/usr/bin/python3
# -*- coding: utf-8 -*-
import ams
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pi
import numpy as np


x_goal = 0.5
y_goal = 0
phi_goal = 0.0

K = 2
K_phi = 1.0
K_gamma = 1

ws_max = 1.0
vs_max = 0.3

D = 0.1207  # distance between front and rear axle


global i
i = 0

# Handle odometry
def handleOdometry(msg):
  x, y, phi = ams.msgToPose(msg.pose.pose)
  gamma = msg.pose.pose.position.z

  
  # euclidian distance to the goal
  error_d = ((x_goal - x)**2 + (y_goal - y)**2)**0.5

  # if close to goal, stop
  if error_d < 0.1:
    vs = 0.0
    ws = 0.0
    msgCmdVel = Twist()
    msgCmdVel.linear.x = vs
    msgCmdVel.angular.z = ws
    pubCmdVel.publish(msgCmdVel)
    return


  # desired velocity
  v = K * error_d

  fi_error = phi_goal - phi
  # normalize angle to [-pi, pi]
  while fi_error > pi:
    fi_error -= 2 * pi
  while fi_error < -pi:
    fi_error += 2 * pi
  
  w = K_phi * fi_error

  w  =  0.5
  v = 0

  
  # control ws
  gamma_goal = np.arctan2(D * w, v)
  ws = K_gamma * (gamma_goal - gamma)


  # control vs
  vs =  ((v**2) + (D * w)**2)**0.5


  # limit velocities
  if w > ws_max:
    w = ws_max
  if v > vs_max:
    v = vs_max

  global i
  if i % 10 == 0:
    print("inputs: x: ", x, " y: ", y, " phi: ", phi, " gamma: ", gamma)
    print("errors: ed: ", error_d, " ef: ", fi_error, " eg: ", gamma_goal - gamma)
    print("outputs: vs: ", vs, " ws: ", ws)
    print("------")
  i += 1

  # Velocity commands message
  msgCmdVel = Twist()
  msgCmdVel.linear.x = vs
  msgCmdVel.angular.z = ws
  # Publish velocity commands
  pubCmdVel.publish(msgCmdVel)



try:
  rospy.init_node('control_line')
  
  # Velocity commands publisher.
  pubCmdVel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
  # Odometry subscriber
  subOdom = rospy.Subscriber('odom', Odometry, handleOdometry)

  rospy.spin()
except KeyboardInterrupt:
    pass
