#!/usr/bin/env python
# imports

from __future__ import print_function

import sys
import time

import cv2
import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
import roslib
import rospy
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String

# global constants
# FIXME fix those velocities

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82
LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.05    

CLOSE_BOTTLE_WIDTH = 100

# additionals

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





class myRobot():
    def __init__(self): 
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.turtlebot3_model = rospy.get_param("model", "burger")
        self.target_linear_vel   = 0.0
        self.target_angular_vel  = 0.0
        self.control_linear_vel  = 0.0
        self.control_angular_vel = 0.0

        self.image_pub = rospy.Publisher("/video_source/raw_2",Image)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/video_source/raw",Image,self.cv_callback)

        moveit_commander.roscpp_initialize(sys.argv) #moveit itself init
        self.moveit_robot = moveit_commander.RobotCommander() # robot’s kinematic model and the robot’s current joint states
        self.scene = moveit_commander.PlanningSceneInterface() #remote interface for getting, setting, and updating the robot’s internal understanding of the surrounding world
        self.arm = moveit_commander.MoveGroupCommander('arm') # name of movegroup 1 is “arm”
        self.gripper = moveit_commander.MoveGroupCommander('gripper') #name of movegroup 2 is “gripper”
        self.arm.set_planning_time(2)

    # ========================== init functions for tasks =======================================
    
    # ========================== helper functions to control robot ==============================

    def close_gripper(self):
        print("[!] CLOSE GRIPPER SIGNAL")
        self.gripper.set_joint_value_target([0.003, 0.003])
        self.gripper.go(wait = True)
        rospy.sleep(2)
        self.gripper.stop()
        self.gripper.clear_pose_targets()
    
    def open_gripper(self):
        print("[!] OPEN GRIPPER SIGNAL")
        self.gripper.set_joint_value_target([0.01, 0.01])
        self.gripper.go(wait = True)
        rospy.sleep(2)
        self.gripper.stop()
        self.gripper.clear_pose_targets()

    def arm_down(self):
        print("[!] ARM DOWN")
        joint_values = self.arm.get_current_joint_values() # How to get joint states
        joint_values[0] = 0.003
        joint_values[1] = 1.27
        joint_values[2] = -0.76
        joint_values[3] = -0.25
        self.arm.go(joints = joint_values, wait = True)
        rospy.sleep(10)     # FIXME change if possible to 7
        self.arm.stop()
        self.arm.clear_pose_targets()

    def arm_up(self):
        print("[!] ARM UP")
        joint_values = self.arm.get_current_joint_values() # How to get joint states
        joint_values[0] = -0
        joint_values[1] = -1.0
        joint_values[2] = 0.3
        joint_values[3] = 0.7
        self.arm.go(joints = joint_values, wait = True)
        rospy.sleep(10)     # FIXME change if possible to 7
        self.arm.stop()
        self.arm.clear_pose_targets()

    def publish_twist(self):
        twist = Twist()
        self.control_linear_vel = makeSimpleProfile(self.control_linear_vel, self.target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
        twist.linear.x = self.control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
        self.control_angular_vel = makeSimpleProfile(self.control_angular_vel, self.target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = self.control_angular_vel
        self.pub.publish(twist)

    def pick_up_bottle(self):
        print("[!] Move arm towards bottle...")
        self.arm_down()
        print("[!] Pick the bottle")
        self.open_gripper()
        self.close_gripper()
        print("[!] Move arm up...")
        self.arm_up()

    def put_down_bottle(self):
        print("[!] Move arm forward...")
        self.arm_down()
        print("[!] Put down the bottle")
        self.open_gripper()
        print("[!] Move arm up...")
        self.arm_up()

    # ========================== subscriber callback functions ==================================

    def cv_callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "rgb8"))
        except CvBridgeError as e:
            print(e)

    # ========================== task1 functions ==============================================
    def init_task1(self):
        self.current_task = 1
        self.box_sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.find_bottle)

    def bottle_found_t1(self):
        self.boxsize_sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.approach_bottle_check_size)
        self.go_forward_to_bottle()
    
    def bottle_in_front_t1(self):
        self.pick_up_bottle()
        self.shutDown()

    # ========================== task2 functions ==============================================

    def init_task2(self):
        self.current_task = 2
        # turn left to around 30-40 degrees
        self.target_angular_vel = checkAngularLimitVelocity(ANG_VEL_STEP_SIZE)
        self.publish_twist()
        rospy.sleep(2)  # FIXME change according to the real time needed to turn

        self.target_angular_vel = 0
        self.publish_twist()

        # start turning to left 
        self.target_angular_vel = checkAngularLimitVelocity(ANG_VEL_STEP_SIZE)
        self.publish_twist()

        # then turn on the subscriber
        self.box_sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.find_bottle)
        
        # if we see the bottle approach it and pick it up
        # turn 180 degrees backwards
        # go forward till something [wall?]
        
    def bottle_found_t2(self):
        self.boxsize_sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.approach_bottle_check_size)
        self.go_forward_to_bottle()
        self.action_start_time = time.time()
        self.action_duration = 0
    
    def bottle_in_front_t2(self):
        self.pick_up_bottle()
        # turning 180 backward
        self.target_angular_vel = checkAngularLimitVelocity(ANG_VEL_STEP_SIZE) #FIXME change velocity
        time.sleep(5)
        # go forward
        self.target_angular_vel = 0.0
        self.target_linear_vel = LIN_VEL_STEP_SIZE
        self.publish_twist()
        time.sleep(self.action_duration) # FIXME fix duration time if needed
        self.put_down_bottle()
        # Now, we are at the base arena, we just put down the bottle there
        # Now, we have to take the next bottle
        if self.task2_finished:
            self.shutDown()
        else:
            self.second_bottle()
        
    def second_bottle(self):
        self.target_angular_vel = checkAngularLimitVelocity(ANG_VEL_STEP_SIZE)
        self.publish_twist()
        rospy.sleep(4) # FIXME change time, or write time.sleep() instead
        self.target_angular_vel = 0
        self.publish_twist()

        # start turning to left 
        self.target_angular_vel = checkAngularLimitVelocity(ANG_VEL_STEP_SIZE)
        self.publish_twist()

        # then turn on the subscriber
        self.box_sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.find_bottle)

    # ========================== other functions ==============================================

    def shutDown(self):
        rospy.signal_shutdown("Shutting down after completing the task")

    def goto_init(self):
        # TODO implement shutdown
        pass

    def find_bottle(self, data):
        for box in data.bounding_boxes:
            print("[!] I see: " + box.Class)
            if box.id == 39:         # if not working change to box.Class == "bottle"
                print("[!] Here is a bottle")
                print("[!] xmin:", box.xmin, "xmax:", box.xmax)
                bottle_center = (box.xmin + box.xmax) // 2
                print("[!] Center:", bottle_center)
                if bottle_center < 600:
                    self.target_angular_vel = checkAngularLimitVelocity(ANG_VEL_STEP_SIZE)
                    self.publish_twist()
                elif bottle_center > 680:
                    self.target_angular_vel = checkAngularLimitVelocity(-ANG_VEL_STEP_SIZE)
                    self.publish_twist()
                else:
                    print("[!] Unregistered from find_bottle [BoundingBoxes]...")
                    self.target_angular_vel = 0.0
                    self.publish_twist() # stop angular velocity
                    self.box_sub.unregister()    # here we see the bottle and it is in the center
                    print("[!] Execute go forward and pick-up...")
                    if self.current_task == 1:
                        self.bottle_found_t1()
                    elif self.current_task == 2:
                        self.bottle_found_t2()
            else:
                print("[!] It is not a bottle")
                self.target_angular_vel = 0.0
                self.publish_twist()


    def approach_bottle_check_size(self, data):
        for box in data.bounding_boxes:
            print("[!] [approach_bottle_check_size] I see: " + box.Class)
            if box.id == 39:         # if not working change to box.Class == "bottle"
                print("[!] [approach_bottle_check_size] Here is a bottle")
                print("[!] [approach_bottle_check_size] xmin:", box.xmin, "xmax:", box.xmax)
                bottle_center = (box.xmin + box.xmax) // 2
                bottle_width = box.xmax - box.xmin
                print("[!] [approach_bottle_check_size] Width:", bottle_width)
                # print("[!] [approach_bottle_check_size] Center:", bottle_center)
                if bottle_width >= CLOSE_BOTTLE_WIDTH:
                    self.target_angular_vel = 0
                    self.target_linear_vel = 0
                    self.publish_twist()

                    if self.current_task == 2:                # FIXME fix task if needed
                        self.action_duration = int(time.time()- self.action_start_time)
                    
                    self.boxsize_sub.unregister()
                    if self.current_task == 1:
                        self.bottle_in_front_t1()
                    elif self.current_task == 2:
                        self.bottle_in_front_t2()
                else:
                    print("[!] [approach_bottle_check_size] Just go forward - small width...")
                    self.target_linear_vel = LIN_VEL_STEP_SIZE
                    self.target_angular_vel = 0.0
                    self.publish_twist()
            else:
                print("[!] [approach_bottle_check_size] I don't see the bottle")
                self.target_angular_vel = 0
                self.target_linear_vel = 0
                self.publish_twist()
                
    
    def go_forward_to_bottle(self):
        # FIXME fix how long should it go, or find distance somehow
        self.target_linear_vel = checkLinearLimitVelocity(self.target_linear_vel + LIN_VEL_STEP_SIZE)
        self.target_angular_vel = 0.0
        self.publish_twist()
        time.sleep(4) # FIXME fix for how long should it go forward
        self.target_linear_vel = 0.0
        self.control_linear_vel = 0.0
        self.publish_twist()
        print("[!] Stopped in front of the bottle")
    





def task1(turtle):     # find and pickup the bottle
    turtle.init_task1()
    rospy.init_node('find_bottle_and_pickup', anonymous = True)
    rospy.on_shutdown(turtle.goto_init)
    rospy.spin()


def task2(turtle):
    turtle.init_task2()
    rospy.init_node('find_bottles_and_bring', anonymous = True)
    rospy.on_shutdown(turtle.goto_init)
    rospy.spin()


def task3():
    # TODO: implement
    pass


def task4():
    # TODO: implement
    pass


def main(args):
    if len(args) == 1:
        print("[ERROR] Usage: final.py <task_no>")
        return

    turtle = myRobot()

    if args[1] == '1':
        task1(turtle)
    elif args[1] == '2':
        task2(turtle)
    elif args[1] == '3':
        task3(turtle)
    elif args[1] == '4':
        task4(turtle)
    else:
        print("[ERROR] Type only 1, 2, 3 or 4")


if __name__ == "__main__":
    main(sys.argv)






# TODO: write scripts to run helper commands like roscore and etc.
# TODO: finish task2
