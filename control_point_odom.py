#!/usr/bin/python3
# -*- coding: utf-8 -*-
import ams
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pi
import numpy as np

# naprej
x_goal = -1.2
# v levo
y_goal = 0.0

# za voddenje v lego 
phi_goal = 0

# fi_ref je smer proti cilju 


# globalno v vs in ws
K_gamma = 9.0

# vodenje v tocko
K = 0.5
K_phi = 3.0

# vodenje v lego
K_phi_goal = 1


ws_max = 30
vs_max = 0.3

D = 0.1207  # distance between front and rear axle


global i
i = 0
global point_reached
point_reached = False

# Handle odometry
def handleOdometry(msg):
  x, y, phi = ams.msgToPose(msg.pose.pose)
  gamma = msg.pose.pose.position.z

  
  # euclidian distance to the goal
  error_d = ((x_goal - x)**2 + (y_goal - y)**2)**0.5
  # print all
  # print("x = ", x)
  # print("y = ", y)
  # print("x_goal = ", x_goal)
  # print("y_goal = ", y_goal)
  # print("(x_goal - x)**2 = ", (x_goal - x)**2)
  # print("(y_goal - y)**2 = ", (y_goal - y)**2)
  # print("error_d = ", error_d)

  global point_reached
  # if close to goal, stop
  if error_d < 0.03 or point_reached:



    # vodenje v lego
    ef = phi_goal - phi
    ef = ams.wrapToPi(ef)

    if abs(ef) < 0.03:
      ef = 0.0
      print("Orientation goal reached.")
    

    w = K_phi_goal * ef

    v = 0.0

    # duble code !!
    
    # control ws
    gamma_goal = np.arctan2(D * w, v)
    razlika_kotov = gamma_goal - gamma
    razlika_kotov = ams.wrapToPi(razlika_kotov)
    ws = K_gamma * (razlika_kotov)


    # control vs
    vs =  ((v**2) + (D * w)**2)**0.5


    # limit velocities
    if w > ws_max:
      w = ws_max
    if v > vs_max:
      v = vs_max

    msgCmdVel = Twist()
    msgCmdVel.linear.x = vs
    msgCmdVel.angular.z = ws
    pubCmdVel.publish(msgCmdVel)
    point_reached = True
    print("Point reached.")


    return

    # omejitev hitrosti ce gamma error prevelik
  
  
  phi_ref = np.arctan2(y_goal - y, x_goal - x)

  fi_error = phi_ref - phi
  fi_error = ams.wrapToPi(fi_error)
  
  w = K_phi * fi_error

  if abs(fi_error) > np.deg2rad(90):
    K_corected = 0
  else:
    K_corected = np.cos(fi_error)**2 * K


  # desired velocity
  v = K_corected * error_d

  # -----------------------------
  # w  = 2
  # v = 0.0
  # OK


  
  # control ws
  gamma_goal = np.arctan2(D * w, v)
  razlika_kotov = gamma_goal - gamma
  razlika_kotov = ams.wrapToPi(razlika_kotov)
  ws = K_gamma * (razlika_kotov)


  # control vs
  vs =  ((v**2) + (D * w)**2)**0.5




  # limit velocities
  if ws > ws_max:
    ws = ws_max
  if vs > vs_max:
    vs = vs_max

  global i
  if i % 10 == 0 or True:
    print("inputs: x: ", x, " y: ", y, " phi: ", phi, " gamma: ", gamma)
    print("errors: ed: ", error_d, " ef: ", fi_error, " eg: ", gamma_goal - gamma)
    print("outputs: vs: ", vs, " ws: ", ws)
    print("------")
  i += 1

  #---------------------------------------
  # vs = 0.0
  # ws = 2.0
  # OK

  # vs = 0.0
  # ws = 0.0

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
