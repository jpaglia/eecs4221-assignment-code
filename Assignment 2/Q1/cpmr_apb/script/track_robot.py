#! /usr/bin/env python3
#
# Question 1.d
import rospy
import math
import sys
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_from_euler

noisy_odom = None

def callback(data):
  global noisy_odom
  noisy_odom = data

def beacon_publisher(noisy_odom, publisher, beacon_id, beacon_x, beacon_y):
	robot_position_x = noisy_odom.pose.pose.position.x
	robot_position_y = noisy_odom.pose.pose.position.y
	
	distance_x = beacon_x - robot_position_x
	distance_y = beacon_y - robot_position_y
	d = math.sqrt(math.pow(distance_x, 2) + math.pow(distance_y, 2))
	
	posMsg = f"source {beacon_id} location ({beacon_x},{beacon_y}) distance {d}"
	publisher.publish(posMsg)
	rospy.loginfo(posMsg)

if __name__ == '__main__':
	myargv = rospy.myargv(argv=sys.argv)
	if len(myargv) != 7:
		print('Execute command as track_robot.py beacon_id x y')
	robot = rospy.get_param("~robot", "block_robot")
	cmdvel = rospy.get_param("~cmdvel", "cmd_vel")
	backup_time = float(rospy.get_param("~backupt", "5.0"))
	velocity = float(rospy.get_param("0.2", "0.2"))
	odomMsg = rospy.get_param("~odom", "odom")
	rate = int(rospy.get_param("~rate", "30"))

	node_name = 'track_robot_' + str(myargv[1])
	rospy.init_node(node_name)  # add in logic for beacon num
	r =rospy.Rate(rate) # in hz
	
	
	rospy.Subscriber('noisy_channel', Odometry, callback)

	# Beacon Publisher
	pub_name = 'beacon_' + str(myargv[1]) + '/range'
	beacon_pub = rospy.Publisher(pub_name, String, queue_size=1)

	try:
		while not rospy.is_shutdown():
			r.sleep()
			posMsg = ""
			
			if noisy_odom != None:
				beacon_publisher(noisy_odom, beacon_pub, myargv[1], int(myargv[2]), int(myargv[3]))
				
		  
	except rospy.ROSInterruptException:
		pass
    

  # pub.publish("Shut down")
	rospy.loginfo(f"Shutting down")

