#!/usr/bin/python3
# -*- coding: utf-8 -*-
import ams
from agvapi import Agv, findLineEdges
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from amsagv_msgs.msg import LineStamped
import numpy as np
import math
import matplotlib.pyplot as plt

PI = 3.14



with Agv() as robot:
  # Handle velocity commands
  def handleCmdVel(msg):
    global robot
    robot.setVel(msg.linear.x, msg.angular.z)



  try:
    rospy.init_node('agv')
    ns = rospy.get_namespace().lstrip('/')
    # Name of the odometry frame
    paramOdomFrameId = rospy.get_param('~odom_frame_id', '{}odom'.format(ns))
    # Name of the AGV frame
    paramAgvFrameId = rospy.get_param('~agv_frame_id', '{}agv'.format(ns))

    # Odometry publisher
    pubOdom = rospy.Publisher('odom', Odometry, queue_size=1)
    # Line sensor publisher
    pubLine = rospy.Publisher('line', LineStamped, queue_size=1)
    # Velocity commands subscriber.
    subCmdVel = rospy.Subscriber('cmd_vel', Twist, handleCmdVel)

    # Line-sensor message
    msgLine = LineStamped()

    # Odometry message
    msgOdom = Odometry()
    msgOdom.header.frame_id = paramOdomFrameId
    msgOdom.child_frame_id = paramAgvFrameId

    # Odometry initial state
    x, y, phi, gamma = 0.0, 0.0, 0.0, 0.0 # Robot configuration
    fd = 0.0 # Travelled distance of the front cart

    
    # Get inital positions
    robot.readSensors()
    encLeft, encRight, encHeading = robot.getEncoders()
  
    prev_pos_l = encLeft
    prev_pos_r = encRight
    
    fi_prev = phi
    x_prev = x
    y_prev = y


    # Position of encoders
    sample_time = 0.02
    i = 0
    rate = rospy.Rate(50) # 20 ms
    while not rospy.is_shutdown():
      t = rospy.Time.now()

      # Read sensors
      robot.readSensors()

      #
      # Odometry
      #

      # Encoders
      encLeft, encRight, encHeading = robot.getEncoders()

      #TODO Implement odometry here ...
      tick_per_m_left = -112043
      tick_per_m_right = 112312
      gamma_offset_deg = 73.1
      cpr = 8192

      vr = ((encRight - prev_pos_r) / sample_time)/ tick_per_m_right
      vl = ((encLeft - prev_pos_l) / sample_time)/ tick_per_m_left

      prev_pos_r = encRight
      prev_pos_l = encLeft


      vs = (vr + vl) / 2

      gamma_offset_rad = np.deg2rad(gamma_offset_deg)
      gamma = -(encHeading/ cpr * 2 * PI - gamma_offset_rad)
      ##TODO: test modulop operator za neg stevila

      #fi_odvod = w
      d = 0.1207
      w = vs/d * np.sin(gamma)
      fi_odvod = w

      # fi 
      phi = fi_prev + fi_odvod * sample_time
      fi_prev = phi

      # x, y derivatives
      x_dot = vs * np.cos(gamma) * np.cos(phi)
      y_dot = vs * np.cos(gamma) * np.sin(phi)

      # x, y
      x = x_prev + x_dot * sample_time
      y = y_prev + y_dot * sample_time

      x_prev = x
      y_prev = y


      #print('Encoders: left={}, right={}, heading={}'.format(encLeft, encRight, encHeading))
      #print(f'V: left={vl}, righ={vr}, vs={vs}')
      print('Odom: x={}, y={}, phi={}, gamma={}'.format(x,y,np.rad2deg(phi),np.rad2deg(gamma)))



      i = i + 1

      # Odometry message
      msgOdom.header.stamp = t
      msgOdom.pose.pose = ams.poseToPoseMsg(x, y, phi)
      msgOdom.pose.pose.position.z = gamma
      # Publish odometry message
      pubOdom.publish(msgOdom)

      #
      # Line sensor
      #

      # Line-sensor values
      lineValues = robot.getLineValues()
      # Left and right line edge
      edgeLeft, edgeRight = findLineEdges(lineValues)

      # Line-sensor message
      msgLine.header.stamp = t
      msgLine.line.values = lineValues
      msgLine.line.left = edgeLeft if edgeLeft is not None else float('nan')
      msgLine.line.right = edgeRight if edgeRight is not None else float('nan')
      msgLine.line.heading = gamma
      msgLine.line.distance = fd
      # Publish line-sensor message
      pubLine.publish(msgLine)

      rate.sleep()
  except KeyboardInterrupt:
    pass



