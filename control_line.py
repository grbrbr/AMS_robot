#!/usr/bin/python3
# -*- coding: utf-8 -*-
import ams
import rospy
from geometry_msgs.msg import Twist
from amsagv_msgs.msg import LineStamped, TagStamped
from math import pi, sin, cos, isnan
from world import MTAG
import math

#math.isnan



tag = None

edgeSelection = "R"
#edgeSelection = "L"


P_val = 3.2

v_default = 0.15

# Handle line sensor
def handleLine(msg):
  #print(msg.line.left, msg.line.right)
  
  #TODO Implement line following control algorithm here ...

  v, w = 0.0, 0.0
  valid = True
  error = 0

  if edgeSelection == "R":
    reference = -0.48
  elif edgeSelection == "L":
    reference = 0.48

  
  if (msg.line.left == 1.0) and (msg.line.right == -1.0):
    valid = False
  
  if edgeSelection == "R":
      lineVal = msg.line.right
  elif edgeSelection == "L":
      lineVal = msg.line.left

  if math.isnan(lineVal):
    valid = False
  

  if valid:
    
    error = -(reference - lineVal)

    #if abs(error) < 0.1:
      #error=0

    w = error * P_val
    v = v_default

  
  
  #print(f"lineval:{lineVal}, error:{error}, w:{w}")

  # Velocity commands message
  msgCmdVel = Twist()
  msgCmdVel.linear.x = v
  msgCmdVel.angular.z = w
  # Publish velocity commands
  pubCmdVel.publish(msgCmdVel)



def handleTag(msg):
  global tag
  global edgeSelection
  tag = MTAG.get(msg.tag.id, None)
  
  if tag == 20:
    edgeSelection ="L" 
  elif tag == 18:
    edgeSelection ="R" 
  elif tag == 17:
    edgeSelection ="L" 
  elif tag == 16:
    edgeSelection ="R"



  print('New tag: {} -> {}'.format(msg.tag.id, tag))





try:
  rospy.init_node('control_line')
  
  # Velocity commands publisher.
  pubCmdVel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
  # Line sensor subscriber
  subLine = rospy.Subscriber('line', LineStamped, handleLine)
  # Tag subscriber
  subTag = rospy.Subscriber('tag', TagStamped, handleTag)

  rospy.spin()
except KeyboardInterrupt:
  pass
