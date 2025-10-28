#!/usr/bin/python3
# -*- coding: utf-8 -*-
from math import *
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point, PointStamped
import numpy as np
from ams import wrapToPi
import tf.transformations as tft

#from threading import Lock

# Estimate vector with x, y and phi
estimate = np.zeros(3, dtype=float)
# Estimate covariance matrix (3 x 3)
estimateCov = np.zeros((3,3), dtype=float)

#mutex = Lock()

def handleGps(msgGps):
  global estimate, estimateCov
  #with mutex:
  print(f'GPS: x = {msgGps.point.x} m, y = {msgGps.point.y} m')


def handleOdometry(msgOdom):
  global estimate, estimateCov
  #with mutex:
  print(f'Odometry: d = {msgOdom.twist.twist.linear.x} m, gamma = {msgOdom.pose.pose.position.z}')

  #TODO Implement localisation algorithm here ...

  # Publish estimate
  msgEstimate = Odometry()
  msgEstimate.header.stamp = msgOdom.header.stamp
  msgEstimate.header.frame_id = 'world'
  msgEstimate.child_frame_id = paramEstimateFrameId
  msgEstimate.pose.pose.position.x = estimate[0]
  msgEstimate.pose.pose.position.y = estimate[1]
  msgEstimate.pose.pose.orientation.z = sin(estimate[2]/2.0)
  msgEstimate.pose.pose.orientation.w = cos(estimate[2]/2.0)
  msgEstimate.pose.covariance[0:2] = estimateCov[0,0:2]
  msgEstimate.pose.covariance[5] = estimateCov[0,2]
  msgEstimate.pose.covariance[6:8] = estimateCov[1,0:2]
  msgEstimate.pose.covariance[11] = estimateCov[1,2]
  msgEstimate.pose.covariance[30:32] = estimateCov[2,0:2]
  msgEstimate.pose.covariance[35] = estimateCov[2,2]
  pubEstimate.publish(msgEstimate)
  
  #'''
  vecTodom2agv = np.array([msgOdom.pose.pose.position.x, msgOdom.pose.pose.position.y, msgOdom.pose.pose.position.z], dtype=float)
  vecQodom2agv = np.array([msgOdom.pose.pose.orientation.x, msgOdom.pose.pose.orientation.y, msgOdom.pose.pose.orientation.z, msgOdom.pose.pose.orientation.w], dtype=float)
  vecTworld2agv = np.array([msgEstimate.pose.pose.position.x, msgEstimate.pose.pose.position.y, msgEstimate.pose.pose.position.z], dtype=float)
  vecQworld2agv = np.array([msgEstimate.pose.pose.orientation.x, msgEstimate.pose.pose.orientation.y, msgEstimate.pose.pose.orientation.z, msgEstimate.pose.pose.orientation.w], dtype=float)
  vecQworld2odom = tft.quaternion_multiply(vecQworld2agv, tft.quaternion_conjugate(vecQodom2agv))
  matRworld2odom = tft.quaternion_matrix(vecQworld2odom)[0:3,0:3]
  vecTworld2odom = vecTworld2agv - matRworld2odom.dot(vecTodom2agv)

  msgTFworld2odom = TransformStamped()
  msgTFworld2odom.header.stamp = msgOdom.header.stamp
  msgTFworld2odom.header.frame_id = 'world'
  msgTFworld2odom.child_frame_id = msgOdom.header.frame_id
  msgTFworld2odom.transform.translation = Point(*vecTworld2odom)
  msgTFworld2odom.transform.rotation = Quaternion(*vecQworld2odom)

  tfBroadcaster.sendTransform(msgTFworld2odom)
  #'''



if __name__ == '__main__':
  try:
    rospy.init_node('localisation')
    ns = rospy.get_namespace().strip('/')
    # Name of the estimate frame
    paramEstimateFrameId = rospy.get_param('~estimate_frame_id', f'{ns}/estimate')

    tfBroadcaster = tf2_ros.TransformBroadcaster()
    
  # Estimate publisher
    pubEstimate = rospy.Publisher('estimate', Odometry, queue_size=1)
  # Odometry subscriber
    subOdom = rospy.Subscriber('odom', Odometry, handleOdometry)
    # GPS subscriber
    subGps = rospy.Subscriber('gps', PointStamped, handleGps)

    rospy.spin()
  except KeyboardInterrupt:
    pass
