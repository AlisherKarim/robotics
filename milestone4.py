#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String, Int64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Twist


WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1


def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def checkLinearLimitVelocity(vel):
    vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
    return vel

def checkAngularLimitVelocity(vel):
    vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
    return vel


class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/video_source/raw_2",Image)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/video_source/raw",Image,self.callback)
    self.sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.callback_fn) 

    self.publisher = rospy.Publisher('box/width', Int64, queue_size = 10)

    self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    self.turtlebot3_model = rospy.get_param("model", "burger")
    self.target_linear_vel   = 0.0
    self.target_angular_vel  = 0.0
    self.control_linear_vel  = 0.0
    self.control_angular_vel = 0.0

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "rgb8"))
    except CvBridgeError as e:
      print(e)

  def callback_fn(self, data):
    for box in data.bounding_boxes:
      print("I see: " + box.Class)
      if box.id == 39 or box.Class == 'vase':         # if not working change to box.Class == "bottle"
        print("Here is a bottle")
        print("xmin:", box.xmin, "xmax:", box.xmax)
        bottle_center = (box.xmin + box.xmax) // 2
        print("Center:", bottle_center)
        print("Width:", box.xmax - box.xmin)
        self.publisher.publish(box.xmax - box.xmin)
         
def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  rospy.spin()
  
if __name__ == '__main__':
    main(sys.argv)
