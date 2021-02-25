#! /usr/bin/env python3
#
# Question 1.a
import rospy
import math
from numpy import random
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_from_euler

noisy_x = 0
noisy_y = 0 
noisy_theta = 0
odom = None
noisy_odom = None
x_old = 0
y_old = 0
theta_old = 0

def odometry_callback(msg):
  global odom, noisy_x, x_old, noisy_y, y_old, noisy_theta, theta_old
  sigma = 0.01
  x_real = msg.pose.pose.position.x
  y_real = msg.pose.pose.position.y
  orientation_q = msg.pose.pose.orientation
  orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
  (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
  theta_real = yaw

  x_diff = x_real - x_old
  y_diff = y_real - y_old
  theta_diff = theta_real - theta_old

  sigma_x = abs(x_diff * sigma)	# 0.01 is the standard deviation diff of 0.01/1m
  noise_x = random.normal(loc=0, scale=sigma_x)

  sigma_y = abs(y_diff * sigma)	# 0.01 is the standard deviation diff of 0.01/1m
  noise_y = random.normal(loc=0, scale=sigma_y)
  
  sigma_theta = abs(theta_diff * sigma)	# 0.01 is the standard deviation diff of 0.01/1m
  noise_theta = random.normal(loc=0, scale=sigma_theta)
  
  x_diff += noise_x
  noisy_x += x_diff
  x_old = x_real

  y_diff += noise_y
  noisy_y += y_diff
  y_old = y_real
  
  theta_diff += noise_theta
  noisy_theta += theta_diff
  theta_old = theta_real

  msg.pose.pose.position.x = noisy_x
  msg.pose.pose.position.y = noisy_y
  msg.pose.pose.orientation.w = noisy_theta
  odom = msg
  

def callback(data):
  global noisy_odom	
  noisy_odom = data

if __name__ == '__main__':
  robot = rospy.get_param("~robot", "block_robot")
  cmdvel = rospy.get_param("~cmdvel", "cmd_vel")
  backup_time = float(rospy.get_param("~backupt", "5.0"))
  velocity = float(rospy.get_param("~velocity", "0.5"))
  odomMsg = rospy.get_param("~odom", "odom")
  rate = int(rospy.get_param("~rate", "30"))


  rospy.init_node('noisy_odom')
  r =rospy.Rate(rate) # in hz
  rospy.Subscriber(odomMsg, Odometry, odometry_callback)
  

  rospy.loginfo(f"Starting to publish on noisy_odom")
  pub = rospy.Publisher('noisy_channel', Odometry, queue_size=1)
  rospy.Subscriber('noisy_channel', Odometry, callback)


  try:
    while not rospy.is_shutdown():
      r.sleep()
      posMsg = ""
      if odom != None:
        # Publishing the odometry message from odom to noisy odom
        # Update the odometry logic to include noise in x, y, and theta
        pub.publish(odom)
      
      if noisy_odom != None:
      	posMsg = f"Noisy Odom at ({noisy_odom.pose.pose.position.x},{noisy_odom.pose.pose.position.y}, {noisy_odom.pose.pose.orientation.w})"
      	rospy.loginfo(posMsg)
      
        
  except rospy.ROSInterruptException:
    pass

  pub.publish("Shut down")
  rospy.loginfo(f"Shutting down")

