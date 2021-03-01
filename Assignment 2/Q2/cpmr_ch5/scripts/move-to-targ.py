#! /usr/bin/env python3
# 
# This code is a modification of drive-to-hit.py from cpmr_ch4 by
#
# Copryight (c) Michael Jenkin, 2021
#
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
#from gazebo_msgs.msg import ContactsState

#contact = None
odom = None

def odometry_callback(msg):
  global odom
  odom = msg

if __name__ == '__main__':
  robot = rospy.get_param("~robot", "block_robot")
  cmdvel = rospy.get_param("~cmdvel", "cmd_vel")
  #backup_time = float(rospy.get_param("~backupt", "5.0"))
  velocity = float(rospy.get_param("~velocity", "0.5"))
  odomMsg = rospy.get_param("~odom", "odom")
  #contactMsg = rospy.get_param("~contact", "robot_bumper_contact_state")
  rate = int(rospy.get_param("~rate", "30"))


  rospy.init_node('drive_to_hit')
  r =rospy.Rate(rate) # in hz
  rospy.Subscriber(odomMsg, Odometry, odometry_callback)
  #rospy.Subscriber(contactMsg, ContactsState, contact_callback)

  rospy.loginfo(f"Starting to publish on {cmdvel}")
  pub = rospy.Publisher(cmdvel, Twist, queue_size=1)
  twist = Twist()
  twist.linear.x = velocity
  pub.publish(twist)
  #current_position = 0.0
  #if(odom.pose.pose.position.x < 0.22 ):
  #	twist.linear.x = velocity
  #	pub.publish(twist)
  	#current_position = odom.pose.pose.position.x
  #	if(odom.pose.pose.position.x >= 0.22):
  #		twist.linear.x = 0
  #		pub.publish(twist)
	#while (odom.pose.pose.position.x < 0.22):
  	#	twist.linear.x = velocity
  	#	pub.publish(twist)
  	#	posMsg = f"Robot at ({odom.pose.pose.position.x},{odom.pose.pose.position.y})"
  	#twist.linear.x = 0
  	#pub.publish(twist)

  try:
    while not rospy.is_shutdown():
      pub.publish(twist)
      r.sleep()
      posMsg = ""
      if odom != None:
      	posMsg = f"Robot at ({odom.pose.pose.position.x},{odom.pose.pose.position.y})"
      	if(float(odom.pose.pose.position.x) >= 0.22):
      		twist.linear.x = 0
      		pub.publish(twist)

      rospy.loginfo(posMsg)
        
  except rospy.ROSInterruptException:
    pass
  twist = Twist()
  pub.publish(twist)
  rospy.loginfo(f"Shutting down")
