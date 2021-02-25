#! /usr/bin/env python3
#
# Question 1.b
import rospy
import math
import csv
from numpy import random
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler

state = 0
noisy_odom = None
odom = None

def noisy_callback(msg):
	global noisy_odom
	noisy_odom = msg
  

def callback(data):
  global odom
  odom = data

if __name__ == '__main__':
  robot = rospy.get_param("~robot", "block_robot")
  cmdvel = rospy.get_param("~cmdvel", "cmd_vel")
  backup_time = float(rospy.get_param("~backupt", "5.0"))
  velocity = float(rospy.get_param("0.2", "0.2"))
  odomMsg = rospy.get_param("~odom", "odom")
  rate = int(rospy.get_param("~rate", "30"))
  angular_velocity = -0.5235988

  rospy.init_node('move_robot')
  r =rospy.Rate(rate) # in hz
  rospy.Subscriber(odomMsg, Odometry, callback)
  

  rospy.loginfo(f"Starting to publish on noisy_odom")
  rospy.Subscriber('noisy_channel', Odometry, noisy_callback)
  
  
  pub_twist = rospy.Publisher(cmdvel, Twist, queue_size=1)

  twist = Twist()
  twist.linear.x = velocity
  twist.angular.z = 0
  pub_twist.publish(twist)
  
  odom_data = {
  	"x": [],
  	"y" : [],
  	"theta": []
  }
  
  noisy_data = {
  	"x": [],
  	"y" : [],
  	"theta": []
  }
  
  append_data = True;
  try:
    while not rospy.is_shutdown():
      pub_twist.publish(twist)
      r.sleep()
      
      posMsg = ""
      if noisy_odom != None and odom != None:
      	posMsg = f"Noisy Odom at {noisy_odom.pose.pose.position.x} VS {odom.pose.pose.position.x}"
      	orientation_q = odom.pose.pose.orientation
      	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
      	(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
      	
      	orientation_q_noisy = noisy_odom.pose.pose.orientation
      	orientation_list_noisy = [orientation_q_noisy.x, orientation_q_noisy.y, orientation_q_noisy.z, orientation_q_noisy.w]
      	(roll_noisy, pitch_noisy, yaw_noisy) = euler_from_quaternion (orientation_list_noisy)
      	
      	if append_data:
      		odom_data["x"].append(round(odom.pose.pose.position.x, 5))
      		odom_data["y"].append(round(odom.pose.pose.position.y, 5))
      		odom_data["theta"].append(round(yaw, 5))
      		
      		noisy_data["x"].append(round(noisy_odom.pose.pose.position.x, 5))
      		noisy_data["y"].append(round(noisy_odom.pose.pose.position.y, 5))
      		noisy_data["theta"].append(round(yaw_noisy, 5))
      	
      	
      	if odom.pose.pose.position.x >= 2 and state == 0:
      		twist.linear.x = 0
      		twist.angular.z = angular_velocity
      		state = 1
      		pub_twist.publish(twist)
      	elif yaw <= -1.575 and state == 1:
      		twist.linear.x = velocity
      		twist.angular.z = 0
      		state = 2
      		pub_twist.publish(twist)
      	elif odom.pose.pose.position.y <= -2 and state == 2:
      		twist.linear.x = 0
      		twist.angular.z = angular_velocity
      		state = 3
      		pub_twist.publish(twist)
      	elif abs(yaw) >= 3.135 and state == 3:
      		twist.angular.z = 0
      		twist.linear.x = velocity
      		state = 4
      		pub_twist.publish(twist)
      	elif odom.pose.pose.position.x <= 0 and state == 4:
      		twist.linear.x = 0
      		twist.angular.z = angular_velocity
      		state = 5
      		pub_twist.publish(twist)
      	elif abs(yaw) <= 1.575 and state == 5:
      		twist.linear.x = velocity
      		twist.angular.z = 0
      		state = 6
      		pub_twist.publish(twist)
      	elif odom.pose.pose.position.y >= 0 and state == 6:
      		twist.linear.x = 0
      		twist.angular.z = angular_velocity
      		state = 7
      		pub_twist.publish(twist)
      	elif yaw <= 0 and state == 7:
      		append_data = False;
      		twist.linear.x = 0
      		twist.angular.z = 0
      		pub_twist.publish(twist)
      		with open('data.csv', 'w') as csv_file:  
      			writer = csv.writer(csv_file)
      			for key, value in odom_data.items():
      				writer.writerow(value)
      			for key, value in noisy_data.items():
      				writer.writerow(value)
    		

  except rospy.ROSInterruptException:
    pass
    
  twist = Twist()
  pub_twist.publish(twist)
  # pub.publish("Shut down")
  rospy.loginfo(f"Shutting down")

