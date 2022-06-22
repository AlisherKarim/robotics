#!/usr/bin/env python

# NOTE maybe try to look at the map as 2d. Like, store robot's x and y positions

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
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from datetime import datetime
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String, Int64

# global constants
# FIXME fix those velocities

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82
LIN_VEL_STEP_SIZE = 0.1
ANG_VEL_STEP_SIZE = 0.05    

CLOSE_BOTTLE_WIDTH = 385
TOO_CLOSE_BOTTLE_WIDTH = 405

PI = 3.1415926535897

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


        self.front_distance = 0
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.get_front_distance)

        self.bottleWidth = 0
        self.boxXmin = 0
        self.boxXmax = 0
        self.box_width = rospy.Subscriber("/box/width", Int64, self.bottle_width, queue_size = 1)
        self.box_xminSub = rospy.Subscriber("/box/xmin", Int64, self.box_xmin_callback, queue_size = 1)
        self.box_xmaxSub = rospy.Subscriber("/box/xmax", Int64, self.box_xmax_callback, queue_size = 1)

    # ========================== init functions for tasks =======================================
    
    # ========================== helper functions to control robot ==============================

    def close_gripper(self):
        print("[!] CLOSE GRIPPER SIGNAL")
        self.gripper.set_joint_value_target([0.003, 0.003])
        self.gripper.go(wait = True)
        rospy.sleep(5)
        self.gripper.stop()
        self.gripper.clear_pose_targets()
        time.sleep(3)
    
    def open_gripper(self):
        print("[!] OPEN GRIPPER SIGNAL")
        self.gripper.set_joint_value_target([0.01, 0.01])
        self.gripper.go(wait = True)
        rospy.sleep(5)
        self.gripper.stop()
        self.gripper.clear_pose_targets()
        time.sleep(3)

    def arm_down(self):
        print("[!] ARM DOWN")
        joint_values = self.arm.get_current_joint_values() # How to get joint states
        joint_values[0] = 0
        joint_values[1] = 1.10
        joint_values[2] = -0.5
        joint_values[3] = 0
        self.arm.go(joints = joint_values, wait = True)
        rospy.sleep(5)     # FIXME change if possible to 7
        self.arm.stop()
        self.arm.clear_pose_targets()
        time.sleep(5)

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
        print("[!] ARM is UP")

    def publish_twist(self):
        twist = Twist()
        twist.linear.x = self.target_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = self.target_angular_vel
        self.pub.publish(twist)

    def pick_up_bottle(self):
        print("[!] Pick the bottle")
        self.open_gripper()
        print("[!] Move arm towards bottle...")
        self.arm_down()
        
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
        self.bottle_in_the_center = False
        self.box_sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.find_bottle, queue_size = 1)

    def bottle_found_t1(self):
        self.boxsize_sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.approach_bottle_check_size, queue_size = 1)
    
    def bottle_in_front_t1(self):
        self.pick_up_bottle()
        print("[!] Wait 3 seconds..")
        time.sleep(3)
        self.shutDown()

    # ========================== task2 functions ==============================================

    def init_task2(self):
        print("[!] Time:", time.time())
        self.task2_finished = False
        self.current_task = 2
        # turn RIGHT to around 30-40 degrees
        self.rotate(50, -10)

        # start turning to left 
        print("[!] Turn left...")
        self.target_angular_vel = checkAngularLimitVelocity(2*ANG_VEL_STEP_SIZE)
        self.publish_twist()

        # then turn on the subscriber
        self.box_sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.find_bottle)
        
        # if we see the bottle approach it and pick it up
        # turn 180 degrees backwards
        # go forward till something [wall?]
        
    def bottle_found_t2(self):
        self.boxsize_sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.approach_bottle_check_size)
        self.go_forward_to_bottle()
        self.action_start_time = rospy.Time.now().to_sec()
        self.action_duration = 0
    
    def bottle_in_front_t2(self):
        self.pick_up_bottle()
        # turning 180 backward
        self.rotate(190, 10)                    # FIXME
        # go forward
        
        self.target_linear_vel = LIN_VEL_STEP_SIZE
        while(self.front_distance > 0.45):
            self.publish_twist()

        self.target_linear_vel = 0
        self.publish_twist()

        print("[!] Put down the bottle")

        self.put_down_bottle()
        # Now, we are at the base arena, we just put down the bottle there
        # Now, we have to take the next bottle
        if self.task2_finished:
            self.shutDown()
        else:
            self.second_bottle()
            self.task2_finished = True
        
    def second_bottle(self):
        self.rotate(60, -30)

        # start turning to left 
        self.target_angular_vel = checkAngularLimitVelocity(-ANG_VEL_STEP_SIZE)
        self.publish_twist()

        # then turn on the subscriber
        self.box_sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.find_bottle)


    # ======================== task3 functions ================================================

    def init_task3(self):
        self.current_task = 3
        # maybe go forward 15 cm
        self.rotate(45, 30)
        self.go_forward(3)
        self.rotate(90, 30)
        self.go_forward(1)
        self.rotate(90)
        self.box_sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.find_bottle)

        
    
    def rotate(self, angle, speed):
        print("[!] [Rotate]", angle)
        angular_speed = abs(speed) * 2 * PI / 360
        relative_angle = angle * 2 * PI / 360
        self.target_linear_vel = 0
        if speed < 0:
            self.target_angular_vel = -angular_speed
        else:
            self.target_angular_vel = angular_speed

        t0 = rospy.Time.now().to_sec()
        current_angle = 0
        
        while(current_angle < relative_angle):
            # print("current_angle:", current_angle)
            self.publish_twist()
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed*(t1-t0)

        print("[!] FINISH ROTATE")
        self.target_angular_vel = 0
        self.publish_twist()


    def go_forward(self, meters):
        current_position = 0
        self.target_linear_vel = LIN_VEL_STEP_SIZE
        t0 = rospy.Time.now().to_sec()

        while current_position < meters:
            print("[!] CURRENT_POSITION:", current_position)
            self.publish_twist()
            t1 = rospy.Time.now().to_sec()
            current_position = self.target_linear_vel * (t1 - t0)

        print("[!] STOP at", meters, "meters")
        self.target_linear_vel = 0
        self.publish_twist()


    def bottle_found_t3(self):
        print("[!] [task3] Bottle found")
        self.distance = 0
        self.t0 = rospy.Time.now().to_sec()
        self.boxsize_sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.check_bottle_size_and_approach)

    def check_bottle_size_and_approach(self, data):
        for box in data.bounding_boxes:
            print("[!] [task3] I see " + box.Class)
            if box.Class == "bottle":
                print("[!] [task3] xmin:", box.xmin, "xmax:", box.xmax)
                bottle_width = box.xmax - box.xmin
                print("[!] [check_bottle_size_and_approach] Width:", bottle_width)
                # print("[!] [approach_bottle_check_size] Center:", bottle_center)
                if bottle_width >= CLOSE_BOTTLE_WIDTH:
                    self.target_angular_vel = 0
                    self.target_linear_vel = 0
                    self.publish_twist()
                    self.distance = (rospy.Time.now().to_sec() - self.t0) * LIN_VEL_STEP_SIZE
                    self.boxsize_sub.unsubscribe()
                    self.bottle_in_front_t3()
                    break
                else:
                    self.target_linear_vel = LIN_VEL_STEP_SIZE
                    self.publish_twist()
            else:
                self.target_angular_vel = 0
                self.target_linear_vel = 0
                self.publish_twist()


    def bottle_in_front_t3(self):
        print("[!] [task3] Go Home...")
        self.pick_up_bottle()
        self.rotate(180, 30)
        self.go_forward(self.distance)
        self.rotate(-90, 30)
        self.go_forward(1)
        self.rotate(-90, 30)
        self.go_forward(3)
        print("[!] [task3] ShutDown() robot...")
        self.shutDown()


    # ========================== task4 functions ==============================================




    # ========================== other functions ==============================================

    def shutDown(self):
        rospy.signal_shutdown("Shutting down after completing the task")

    def goto_init(self):
        print("[INIT]")
        # TODO implement shutdown


    def bottle_width(self, data):
        self.bottleWidth = int(str(data.data))

    def box_xmin_callback(self, data):
        self.boxXmin = int(str(data.data))

    def box_xmax_callback(self, data):
        self.boxXmax = int(str(data.data))

    def get_front_distance(self, data):
        # print(data.ranges[0], data.ranges[89], data.ranges[179], data.ranges[269], data.ranges[359])
        if data.ranges[0] > 0:
            self.front_distance = data.ranges[0]

    def check_bottle_center(self, box):
        bottle_center = (self.boxXmax + self.boxXmin) // 2
        print("[!] [CHECK CENTER] Center:", bottle_center)
        if bottle_center < 600:
            self.target_linear_vel = 0
            self.target_angular_vel = checkAngularLimitVelocity(ANG_VEL_STEP_SIZE/2)
            self.publish_twist()
            return False
        elif bottle_center > 690:
            self.target_linear_vel = 0
            self.target_angular_vel = checkAngularLimitVelocity(-ANG_VEL_STEP_SIZE/2)
            self.publish_twist()
            return False
        else:
            self.target_angular_vel = 0.0
            self.publish_twist()
            return True



    def check_bottle_center_close(self, box):
        bottle_center = (self.boxXmax + self.boxXmin) // 2
        print("[!] [CHECK CENTER] Center:", bottle_center)
        if bottle_center < 645:
            self.target_linear_vel = 0
            self.target_angular_vel = checkAngularLimitVelocity(ANG_VEL_STEP_SIZE/2)
            self.publish_twist()
            return False
        elif bottle_center > 670:
            self.target_linear_vel = 0
            self.target_angular_vel = checkAngularLimitVelocity(-ANG_VEL_STEP_SIZE/2)
            self.publish_twist()
            return False
        else:
            self.target_angular_vel = 0.0
            self.publish_twist()
            return True



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
                elif bottle_center > 690:
                    self.target_angular_vel = checkAngularLimitVelocity(-ANG_VEL_STEP_SIZE)
                    self.publish_twist()
                else:
                    print("[!] Bottle is in the center")
                    print("[!] Unregistered from find_bottle [BoundingBoxes]...")
                    self.target_angular_vel = 0.0
                    self.publish_twist() # stop angular velocity
                    self.box_sub.unregister()    # here we see the bottle and it is in the center
                    print("[!] Execute go forward and pick-up...")
                    if self.current_task == 1:
                        self.bottle_found_t1()
                    elif self.current_task == 2:
                        self.bottle_found_t2()
                    elif self.current_task == 3:
                        self.bottle_found_t3()
                    break
            else:
                print("[!] It is not a bottle")
                # self.publish_twist()


    def approach_bottle_check_size(self, data):
        pass
        for box in data.bounding_boxes:
            print("[!] [approach_bottle_check_size] I see: " + box.Class)
            if box.id == 39 or box.Class == 'vase':         # if not working change to box.Class == "bottle"

                if not self.check_bottle_center(box):
                    break

                print("[!] [approach_bottle_check_size] Here is a bottle")
                print("[!] [approach_bottle_check_size] xmin:", box.xmin, "xmax:", box.xmax)
                bottle_width = box.xmax - box.xmin
                bottle_center = (box.xmin + box.xmax) // 2
                print("[!] [approach_bottle_check_size] Width:", bottle_width)
                print("[!] [approach_bottle_check_size] bottleWidth:", self.bottleWidth)
                print("[!] [approach_bottle_check_size] Center:", bottle_center)

                if self.bottleWidth >= TOO_CLOSE_BOTTLE_WIDTH:
                    print("[!] TOO CLOSE...")
                    self.target_linear_vel = -LIN_VEL_STEP_SIZE / 4
                    self.publish_twist()
                    break

                elif self.bottleWidth >= CLOSE_BOTTLE_WIDTH:
                    print("[!] Close Enough")

                    if not self.check_bottle_center_close(box):
                        break

                    self.target_angular_vel = 0
                    self.target_linear_vel = LIN_VEL_STEP_SIZE / 4
                    self.publish_twist()

                    rospy.sleep(2)

                    # self.target_angular_vel = 0
                    # self.target_linear_vel = 0
                    # self.publish_twist()

                    print("[!] STOP SIGNAL")

                    if self.current_task == 2:                # FIXME fix task if needed
                        self.action_duration = int(rospy.Time.now().to_sec() - self.action_start_time)
                    
                    self.boxsize_sub.unregister()
                    if self.current_task == 1:
                        self.bottle_in_front_t1()
                    elif self.current_task == 2:
                        self.bottle_in_front_t2()
                    break

                else:
                    print("[!] [approach_bottle_check_size] Just go forward - small width...")
                    self.target_linear_vel = LIN_VEL_STEP_SIZE / 2
                    self.target_angular_vel = 0.0
                    self.publish_twist()
                    break
            else:
                print("[!] [approach_bottle_check_size] I don't see the bottle")
                self.target_angular_vel = 0
                self.target_linear_vel = 0
                self.publish_twist()
                
    
    def go_forward_to_bottle(self):
        # FIXME fix how long should it go, or find distance somehow
        print("[!] [go_forward_to_bottle] ...")
        self.target_linear_vel = checkLinearLimitVelocity(self.target_linear_vel + LIN_VEL_STEP_SIZE)
        self.target_angular_vel = 0.0
        self.publish_twist()
    





def task1(turtle):     # find and pickup the bottle
    rospy.init_node('find_bottle_and_pickup', anonymous = True)
    turtle.init_task1()
    rospy.on_shutdown(turtle.goto_init)
    rospy.spin()


def task2(turtle):
    print("[STARTING TASK 2]")
    rospy.init_node('find_bottles_and_bring', anonymous = True)
    turtle.init_task2()
    rospy.on_shutdown(turtle.goto_init)
    rospy.spin()


def task3(turtle):
    rospy.init_node('find_bottles_from_center_box_and_bring', anonymous = True)
    turtle.init_task3()
    rospy.on_shutdown(turtle.goto_init)
    rospy.spin()


def task4():
    # TODO: implement
    pass


def main(args):
    if len(args) == 1:
        print("[ERROR] Usage: final.py <task_no>")
        return

    turtle = myRobot()
    print("[!] Start...")

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
