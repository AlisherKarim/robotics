#!/usr/bin/env python

# NOTE maybe try to look at the map as 2d. Like, store robot's x and y positions

# imports

from __future__ import print_function

import sys
import time
import turtle

import cv2
import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
import roslib
import rospy
from cv_bridge import CvBridge, CvBridgeError
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

        moveit_commander.roscpp_initialize(sys.argv) #moveit itself init
        self.moveit_robot = moveit_commander.RobotCommander() # robot’s kinematic model and the robot’s current joint states
        self.scene = moveit_commander.PlanningSceneInterface() #remote interface for getting, setting, and updating the robot’s internal understanding of the surrounding world
        self.arm = moveit_commander.MoveGroupCommander('arm') # name of movegroup 1 is “arm”
        self.gripper = moveit_commander.MoveGroupCommander('gripper') #name of movegroup 2 is “gripper”
        self.arm.set_planning_time(2)


        self.front_distance = 0
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.get_front_distance)

        self.bottle_width = 0
        self.bottle_xmin = 0
        self.bottle_xmax = 0
        self.bottle_item = False
        self.box_widthSub = rospy.Subscriber("/box/width", Int64, self.box_width, queue_size = 1)
        self.box_xminSub = rospy.Subscriber("/box/xmin", Int64, self.box_xmin_callback, queue_size = 1)
        self.box_xmaxSub = rospy.Subscriber("/box/xmax", Int64, self.box_xmax_callback, queue_size = 1)
        self.box_itemSub = rospy.Subscriber("/box/xmax", Int64, self.box_item_callback, queue_size = 1)
    
    # ========================== helper functions to control robot ==============================

    def close_gripper(self):
        print("[!] CLOSE GRIPPER SIGNAL")
        self.gripper.set_joint_value_target([0.003, 0.003])
        self.gripper.go(wait = True)
        rospy.sleep(5)
        self.gripper.stop()
        self.gripper.clear_pose_targets()
    
    def open_gripper(self):
        print("[!] OPEN GRIPPER SIGNAL")
        self.gripper.set_joint_value_target([0.01, 0.01])
        self.gripper.go(wait = True)
        rospy.sleep(5)
        self.gripper.stop()
        self.gripper.clear_pose_targets()

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

    def arm_up(self):
        print("[!] ARM UP")
        joint_values = self.arm.get_current_joint_values() # How to get joint states
        joint_values[0] = -0
        joint_values[1] = -1.0
        joint_values[2] = 0.3
        joint_values[3] = 0.7
        self.arm.go(joints = joint_values, wait = True)
        rospy.sleep(5)     # FIXME change if possible to 7
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

    # ========================== task1 functions ==============================================
    def init_task1(self):

        self.go_forward(1)

        self.current_task = 1
        self.bottle_in_the_center = False

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            box_center = (self.bottle_xmax + self.bottle_xmin) // 2
            if box_center < 630:
                self.target_angular_vel = -ANG_VEL_STEP_SIZE
            elif box_center > 670:
                self.target_angular_vel = ANG_VEL_STEP_SIZE
            else:
                self.target_angular_vel = 0
            
            if self.bottle_width >= TOO_CLOSE_BOTTLE_WIDTH:
                self.target_linear_vel = -LIN_VEL_STEP_SIZE
            elif self.bottle_width < CLOSE_BOTTLE_WIDTH:
                self.target_linear_vel = LIN_VEL_STEP_SIZE
            else:
                self.target_linear_vel = 0

            self.publish_twist()

            if self.target_angular_vel == 0 and self.target_linear_vel == 0:
                break
            rate.sleep()
        
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
        self.rotate(60, -10)

        # start turning to left 
        print("[!] Turn left...")
        
        rate = rospy.Rate(10)

        # rotate while bottle not seen
        self.target_angular_vel = ANG_VEL_STEP_SIZE

        while not rospy.is_shutdown():
            if self.bottle_item:
                self.target_angular_vel = 0
                self.publish_twist()
                break
            self.publish_twist()
            rate.sleep()

        while not rospy.is_shutdown():
            box_center = (self.bottle_xmax + self.bottle_xmin) // 2
            if box_center < 630:
                self.target_angular_vel = -ANG_VEL_STEP_SIZE
            elif box_center > 670:
                self.target_angular_vel = ANG_VEL_STEP_SIZE
            else:
                self.target_angular_vel = 0
            
            if self.bottle_width >= TOO_CLOSE_BOTTLE_WIDTH:
                self.target_linear_vel = -LIN_VEL_STEP_SIZE
            elif self.bottle_width < CLOSE_BOTTLE_WIDTH:
                self.target_linear_vel = LIN_VEL_STEP_SIZE
            else:
                self.target_linear_vel = 0

            self.publish_twist()

            if self.target_angular_vel == 0 and self.target_linear_vel == 0:
                break
            rate.sleep()

        self.bottle_in_front_t2()
    
    def bottle_in_front_t2(self):
        self.pick_up_bottle()
        # turning 180 backward
        self.rotate(190, 10)                    # FIXME
        # go forward
        
        self.target_linear_vel = LIN_VEL_STEP_SIZE
        while(self.front_distance > 0.45):      # FIXME, maybe add rate.sleep()?
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
        # turn RIGHT to around 30-40 degrees
        self.rotate(190, -10)

        # start turning to left 
        print("[!] Turn left...")
        
        rate = rospy.Rate(10)

        # rotate while bottle not seen
        self.target_angular_vel = ANG_VEL_STEP_SIZE

        while not rospy.is_shutdown():
            if self.bottle_item:
                self.target_angular_vel = 0
                self.publish_twist()
                break
            self.publish_twist()
            rate.sleep()

        while not rospy.is_shutdown():
            box_center = (self.bottle_xmax + self.bottle_xmin) // 2
            if box_center < 630:
                self.target_angular_vel = -ANG_VEL_STEP_SIZE
            elif box_center > 670:
                self.target_angular_vel = ANG_VEL_STEP_SIZE
            else:
                self.target_angular_vel = 0
            
            if self.bottle_width >= TOO_CLOSE_BOTTLE_WIDTH:
                self.target_linear_vel = -LIN_VEL_STEP_SIZE
            elif self.bottle_width < CLOSE_BOTTLE_WIDTH:
                self.target_linear_vel = LIN_VEL_STEP_SIZE
            else:
                self.target_linear_vel = 0

            self.publish_twist()

            if self.target_angular_vel == 0 and self.target_linear_vel == 0:
                break
            rate.sleep()

        self.task2_finished = True
        self.bottle_in_front_t2()

    # ======================== task3 functions ================================================

    def init_task3(self):
        self.current_task = 3
        # maybe go forward 15 cm
        self.rotate(40, 10)
        # self.go_forward(3)

        r = rospy.Rate(10)

        self.target_linear_vel = LIN_VEL_STEP_SIZE
        while(self.front_distance > 0.45):      # FIXME
            self.publish_twist()
            r.sleep()

        self.target_linear_vel = 0
        self.publish_twist()

        self.rotate(90, -10)
        self.go_forward(1)                      # FIXME
        self.rotate(90)

        while not rospy.is_shutdown():
            box_center = (self.bottle_xmax + self.bottle_xmin) // 2
            if box_center < 630:
                self.target_angular_vel = -ANG_VEL_STEP_SIZE
            elif box_center > 670:
                self.target_angular_vel = ANG_VEL_STEP_SIZE
            else:
                self.target_angular_vel = 0
            
            if self.bottle_width >= TOO_CLOSE_BOTTLE_WIDTH:
                self.target_linear_vel = -LIN_VEL_STEP_SIZE
            elif self.bottle_width < CLOSE_BOTTLE_WIDTH:
                self.target_linear_vel = LIN_VEL_STEP_SIZE
            else:
                self.target_linear_vel = 0

            self.publish_twist()

            if self.target_angular_vel == 0 and self.target_linear_vel == 0:
                break
            r.sleep()

        self.pick_up_bottle()
        
        self.rotate(190, 10)
        self.rotate(90, 10)
        
        self.target_linear_vel = LIN_VEL_STEP_SIZE
        while(self.front_distance > 0.45):      # FIXME
            self.publish_twist()
            r.sleep()

        self.target_linear_vel = 0
        self.publish_twist()

        self.rotate(90, 10)

        self.target_linear_vel = LIN_VEL_STEP_SIZE
        while(self.front_distance > 0.45):      # FIXME
            self.publish_twist()
            r.sleep()

        self.target_linear_vel = 0
        self.publish_twist()

        self.put_down_bottle()

    
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


    # ========================== task4 functions ==============================================

    def init_task4(self):
        # turn RIGHT to around 30-40 degrees
        self.rotate(60, -10)

        # start turning to left 
        print("[!] Turn left...")
        
        rate = rospy.Rate(10)

        # rotate while bottle not seen
        self.target_angular_vel = ANG_VEL_STEP_SIZE

        while not rospy.is_shutdown():
            if self.bottle_item:
                self.target_angular_vel = 0
                self.publish_twist()
                break
            self.publish_twist()
            rate.sleep()

        while not rospy.is_shutdown():
            box_center = (self.bottle_xmax + self.bottle_xmin) // 2
            if box_center < 630:
                self.target_angular_vel = -ANG_VEL_STEP_SIZE
            elif box_center > 670:
                self.target_angular_vel = ANG_VEL_STEP_SIZE
            else:
                self.target_angular_vel = 0
            
            if self.bottle_width >= TOO_CLOSE_BOTTLE_WIDTH:
                self.target_linear_vel = -LIN_VEL_STEP_SIZE
            elif self.bottle_width < CLOSE_BOTTLE_WIDTH:
                self.target_linear_vel = LIN_VEL_STEP_SIZE
            else:
                self.target_linear_vel = 0

            self.publish_twist()

            if self.target_angular_vel == 0 and self.target_linear_vel == 0:
                break
            rate.sleep()

        self.pick_up_bottle()
        self.rotate(190, 10)

        r = rospy.Rate(10)

        self.target_linear_vel = LIN_VEL_STEP_SIZE
        while(self.front_distance > 0.45):      # FIXME
            self.publish_twist()
            r.sleep()

        self.target_linear_vel = 0
        self.publish_twist()

        self.rotate(90, -10)                    # FIXME

        r = rospy.Rate(10)

        self.target_linear_vel = LIN_VEL_STEP_SIZE
        while(self.front_distance > 0.45):      # FIXME
            self.publish_twist()
            r.sleep()

        self.target_linear_vel = 0
        self.publish_twist()

        self.put_down_bottle()

        self.rotate(90, -10)
        self.go_forward(1)                      # FIXME
        self.rotate(90)

        while not rospy.is_shutdown():
            box_center = (self.bottle_xmax + self.bottle_xmin) // 2
            if box_center < 630:
                self.target_angular_vel = -ANG_VEL_STEP_SIZE
            elif box_center > 670:
                self.target_angular_vel = ANG_VEL_STEP_SIZE
            else:
                self.target_angular_vel = 0
            
            if self.bottle_width >= TOO_CLOSE_BOTTLE_WIDTH:
                self.target_linear_vel = -LIN_VEL_STEP_SIZE
            elif self.bottle_width < CLOSE_BOTTLE_WIDTH:
                self.target_linear_vel = LIN_VEL_STEP_SIZE
            else:
                self.target_linear_vel = 0

            self.publish_twist()

            if self.target_angular_vel == 0 and self.target_linear_vel == 0:
                break
            rate.sleep()

        self.pick_up_bottle()
        
        self.rotate(190, 10)
        self.rotate(90, 10)
        
        self.target_linear_vel = LIN_VEL_STEP_SIZE
        while(self.front_distance > 0.45):      # FIXME
            self.publish_twist()
            r.sleep()

        self.target_linear_vel = 0
        self.publish_twist()

        self.rotate(90, 10)

        self.target_linear_vel = LIN_VEL_STEP_SIZE
        while(self.front_distance > 0.45):      # FIXME
            self.publish_twist()
            r.sleep()

        self.target_linear_vel = 0
        self.publish_twist()

        self.put_down_bottle()

    # ========================== other functions ==============================================

    def shutDown(self):
        self.pub.publish(Twist())
        rospy.signal_shutdown("Shutting down after completing the task")

    def goto_init(self):
        print("[INIT]")
        self.pub.publish(Twist())
        # TODO implement shutdown


    # ===================================== callbacks ===========================================

    def box_width(self, data):
        self.bottle_width = int(str(data.data))

    def box_xmin_callback(self, data):
        self.bottle_xmin = int(str(data.data))

    def box_xmax_callback(self, data):
        self.bottle_xmax = int(str(data.data))

    def box_item_callback(self, data):
        if str(data.data) == "bottle":
            self.bottle_item = True
        else:
            self.bottle_item = False

    def get_front_distance(self, data):
        # print(data.ranges[0], data.ranges[89], data.ranges[179], data.ranges[269], data.ranges[359])
        if data.ranges[0] > 0:
            self.front_distance = data.ranges[0]

    # ============================================================================================
    





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


def task4(turtle):
    rospy.init_node('find_put_find_return', anonymous=True)
    turtle.init_task4()
    rospy.on_shutdown(turtle.goto_init)
    rospy.spin()


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
