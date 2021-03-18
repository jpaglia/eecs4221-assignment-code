#!/usr/bin/env python3
import rospy
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from keras.preprocessing.image import img_to_array
from keras.models import load_model

class FollowRoad:
  _FORWARD = 0
  _TURNING_RIGHT = 1
  _TURNING_LEFT = 2
  
  def __init__(self, image_topic, cmd_topic, x_vel, theta_vel, output, image_size):
    self._bridge = cv_bridge.CvBridge()
    self._image_sub = rospy.Subscriber(image_topic, Image, self._image_callback)
    self._pub = rospy.Publisher(cmd_topic, Twist, queue_size = 1)
    self._x_vel = x_vel
    self._theta_vel = theta_vel
    self._output = output
    self._image_size = image_size
    self._left_id = 0
    self._right_id = 0
    self._forward_id = 0
    self._model = load_model("model")
    self._started = False

  def _image_callback(self, msg):
    image = self._bridge.imgmsg_to_cv2(msg, "bgr8")
    cv2.imshow('window', image)
    image = cv2.resize(image, (28, 28))
    key = cv2.waitKey(3)
    im = img_to_array(image)
    im = np.array(im, dtype="float") / 255.0
    im = im.reshape(-1, 28, 28, 3)
    prediction = np.argmax(self._model.predict(im))
    #print(prediction)

    if key == 113:
    	self.stop()
    	self._started = False
    elif key == 103 or self._started:
    	self._started = True
    	if prediction == FollowRoad._FORWARD:
    		self.go_straight()
    		print("Going Straight\n")
    	elif prediction == FollowRoad._TURNING_LEFT:
    		self.turn_left()
    		print("Turning Left\n")
    	elif prediction == FollowRoad._TURNING_RIGHT:
    		self.turn_right()
    		print("Turning Right\n")
      

  def _command(self, x_vel, theta_vel):
    twist = Twist()
    twist.linear.x = x_vel
    twist.angular.z = theta_vel
    self._pub.publish(twist)

  def go_straight(self):
    self._command(self._x_vel, 0)

  def turn_left(self):
    self._command(self._x_vel, self._theta_vel)

  def turn_right(self):
    self._command(self._x_vel, -self._theta_vel)

  def stop(self):
    self._command(0, 0)

if __name__ == '__main__':
  rospy.init_node('follow_road')
  image_topic = rospy.get_param("~image", "/mycamera/image_raw")
  cmd_topic = rospy.get_param("~cmd", "cmd_vel")
  hz = int(rospy.get_param("~rate", 10))
  output = rospy.get_param("~output", "database")
  x_vel = float(rospy.get_param("~x_vel", 0.2))
  theta_vel = float(rospy.get_param("~theta_vel", 0.2))
  image_size = int(rospy.get_param("~size", 28))

  camera = FollowRoad(image_topic, cmd_topic, x_vel, theta_vel, output, image_size)
  rospy.spin()

