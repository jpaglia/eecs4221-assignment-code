#! /usr/bin/env python3
#
# Question 1.d
import rospy
import math
import sys
import csv
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_from_euler

beacon_0 = None
beacon_1 = None
beacon_2 = None
count = 0

pred_data = {
	"x": [],
	"y" : []
}

def parse_beacon_string(data):
	parsed_data = str(data).split(' ')
	pos = parsed_data[4]
	parsed_position = pos.split(',')
	pos_x = float(parsed_position[0][1:])
	pos_y = float(parsed_position[1][:-1])
	
	dist = float(parsed_data[6][:-1])
	
	result = {
		'position': (pos_x, pos_y),
		'distance': dist
	}
	
	return result
	
def callback_beacon_0(data):
  global beacon_0
  beacon_0 = parse_beacon_string(data)
  
def callback_beacon_1(data):
  global beacon_1
  beacon_1 = parse_beacon_string(data)
  
def callback_beacon_2(data):
  global beacon_2
  beacon_2 = parse_beacon_string(data)


def find_intersect():
	pos_x = (math.pow(beacon_0['distance'], 2) - math.pow(beacon_1['distance'], 2) - math.pow(beacon_0['position'][0], 2) + math.pow(beacon_1['position'][0], 2))/(2*(beacon_1['position'][0] - beacon_0['position'][0]))
	pos_y = math.sqrt(math.pow(beacon_0['distance'], 2) - math.pow((pos_x - beacon_0['position'][0]), 2)) - beacon_0['position'][1]
	
	d_pred_1 = math.sqrt(math.pow(pos_x - beacon_2['position'][0], 2) + math.pow(pos_y - beacon_2['position'][1], 2))
	d_pred_2 = math.sqrt(math.pow(pos_x - beacon_2['position'][0], 2) + math.pow(-pos_y - beacon_2['position'][1], 2))
	
	
	diff_1 = abs(beacon_2['distance'] - d_pred_1)
	diff_2 = abs(beacon_2['distance'] - d_pred_2)
	
	if diff_1 < diff_2:
		print('The robot is predicted to be at X: ' + str(round(pos_x, 5)) + ' Y: ' + str(round(pos_y, 5)))
		pred_data["x"].append(round(pos_x, 5))
		pred_data["y"].append(round(pos_y, 5))	
	else:
		print('The robot is predicted to be at X: ' + str(round(pos_x, 5)) + ' Y: ' + str(round(-pos_y, 5)))
		pred_data["x"].append(round(pos_x, 5))
		pred_data["y"].append(round(-pos_y, 5))

if __name__ == '__main__':
	
	robot = rospy.get_param("~robot", "block_robot")
	cmdvel = rospy.get_param("~cmdvel", "cmd_vel")
	backup_time = float(rospy.get_param("~backupt", "5.0"))
	
	velocity = float(rospy.get_param("0.2", "0.2"))
	
	rate = int(rospy.get_param("~rate", "30"))


	rospy.init_node('estimate_odom')
	r =rospy.Rate(rate) # in hz

	# Beacon Subscribers
	rospy.Subscriber('beacon_0/range', String,  callback_beacon_0)
	rospy.Subscriber('beacon_1/range', String,  callback_beacon_1)
	rospy.Subscriber('beacon_2/range', String,  callback_beacon_2)
	
  
  
	append_data = True
	try:
		while not rospy.is_shutdown():
			r.sleep()
			posMsg = ""
			
			if beacon_0 != None and beacon_1 != None and beacon_2 != None:
				count += 1
				find_intersect()
				beacon_0 = None
				beacon_1 = None
				beacon_2 = None
				
				if (count > 400 and append_data):
					print('printed')
					append_data = False
					with open('data.csv', 'w') as csv_file:  
						writer = csv.writer(csv_file)
						for key, value in pred_data.items():
							writer.writerow(value)
		   	
		    
		  
	except rospy.ROSInterruptException:
		pass
    

  # pub.publish("Shut down")
	rospy.loginfo(f"Shutting down")

