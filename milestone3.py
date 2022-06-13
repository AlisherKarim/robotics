#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys, select, os, time
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
if os.name == 'nt':
  import msvcrt, time
else:
  import tty, termios

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

msg = """
Control Your TurtleBot3 and OpenManipulatorX!
---------------------------
8 : increase linear velocity
2 : decrease linear velocity
4 : increase angular velocity
6 : decrease angular velocity
5 : base stop


g : gripper open
f : gripper close


0 : Extend arm forward
1 : home pose


Ctrl+C or q to quit
"""

e = """
Communications Failed
"""

def getKey():
    if os.name == 'nt':
        timeout = 0.1
        startTime = time.time()
        while(1):
            if msvcrt.kbhit():
                if sys.version_info[0] >= 3:
                    return msvcrt.getch().decode()
                else:
                    return msvcrt.getch()
            elif time.time() - startTime > timeout:
                return ''

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key





def vels(target_linear_vel, target_angular_vel):
    return "Present Linear Velocity: %s, Angular Velocity: %s " % (target_linear_vel,target_angular_vel)

def joints(joint):
    return "Present Joint Angle J1: %s J2: %s J3: %s J4: %s " % (joint[0], joint[1], joint[2], joint[3])

def kinematics(pose):
    return "Present Kinematics Position X: %s Y: %s Z: %s " % (pose[0], pose[1], pose[2])    






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
    return constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)


def checkAngularLimitVelocity(vel):
    return constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)


if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('turtlebot3_teleop')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    turtlebot3_model = rospy.get_param("model", "burger")

    status = 0
    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_angular_vel = 0.0

    # intialize manipulator

    moveit_commander.roscpp_initialize(sys.argv) #moveit itself init
    robot = moveit_commander.RobotCommander() # robot’s kinematic model and the robot’s current joint states
    scene = moveit_commander.PlanningSceneInterface() #remote interface for getting, setting, and updating the robot’s internal understanding of the surrounding world
    arm = moveit_commander.MoveGroupCommander('arm') # name of movegroup 1 is “arm”
    gripper = moveit_commander.MoveGroupCommander('gripper') #name of movegroup 2 is “gripper”
    arm.set_planning_time(2) # Thats the value TA likes to use
    # gripper.set_planning_time(2)

    try:
        print(msg)
        while not rospy.is_shutdown():
            
            key = getKey()
            if key == '8' :
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
                status = status + 5
                print("===================================================")
                print(vels(target_linear_vel,target_angular_vel))
                print(joints(arm.get_current_joint_values()))
                print(kinematics([arm.get_current_pose().pose.position.x, arm.get_current_pose().pose.position.y, arm.get_current_pose().pose.position.z]))
                print("===================================================")
            elif key == '2' :
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
                status = status + 5
                print("===================================================")
                print(vels(target_linear_vel,target_angular_vel))
                print(joints(arm.get_current_joint_values()))
                print(kinematics([arm.get_current_pose().pose.position.x, arm.get_current_pose().pose.position.y, arm.get_current_pose().pose.position.z]))
                print("===================================================")
            elif key == '4' :
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
                status = status + 5
                print("===================================================")
                print(vels(target_linear_vel,target_angular_vel))
                print(joints(arm.get_current_joint_values()))
                print(kinematics([arm.get_current_pose().pose.position.x, arm.get_current_pose().pose.position.y, arm.get_current_pose().pose.position.z]))
                print("===================================================")
            elif key == '6' :
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
                status = status + 5
                print("===================================================")
                print(vels(target_linear_vel,target_angular_vel))
                print(joints(arm.get_current_joint_values()))
                print(kinematics([arm.get_current_pose().pose.position.x, arm.get_current_pose().pose.position.y, arm.get_current_pose().pose.position.z]))
                print("===================================================")
            elif key == ' ' or key == '5' :
                status = status + 5
                target_linear_vel   = 0.0
                control_linear_vel  = 0.0
                target_angular_vel  = 0.0
                control_angular_vel = 0.0
                print("===================================================")
                print(vels(target_linear_vel,target_angular_vel))
                print(joints(arm.get_current_joint_values()))
                print(kinematics([arm.get_current_pose().pose.position.x, arm.get_current_pose().pose.position.y, arm.get_current_pose().pose.position.z]))
                print("===================================================")
            elif key == '0':
                status = status + 5
                joint_values = arm.get_current_joint_values() # How to get joint states
                joint_values[0] = 0.003
                joint_values[1] = 1.27
                joint_values[2] = -0.76
                joint_values[3] = -0.25

                print("Init Position...")


                arm.go(joints = joint_values, wait = True)

                print("[!] Wait until execution of arm finishes...")
                rospy.sleep(5)
                # time.sleep(6)

                arm.stop()
                arm.clear_pose_targets()
                
                # time.sleep(1)

                print("[!] Execution of Init Pose finished")
                print("===================================================")
                print(vels(target_linear_vel,target_angular_vel))
                print(joints(arm.get_current_joint_values()))
                print(kinematics([arm.get_current_pose().pose.position.x, arm.get_current_pose().pose.position.y, arm.get_current_pose().pose.position.z]))
                print("===================================================")


            elif key == '1':
                status = status + 5
                joint_values = arm.get_current_joint_values() # How to get joint states
                joint_values[0] = -0
                joint_values[1] = -1.0
                joint_values[2] = 0.3
                joint_values[3] = 0.7

                print("Home Position...")


                arm.go(joints = joint_values, wait = True)

                print("[!] Wait until execution of arm finishes...")
                rospy.sleep(5)
                # time.sleep(6)

                arm.stop()
                arm.clear_pose_targets()
                
                # time.sleep(1)

                print("[!] Execution of Home Pose finished...")
                print("===================================================")
                print(vels(target_linear_vel,target_angular_vel))
                print(joints(arm.get_current_joint_values()))
                print(kinematics([arm.get_current_pose().pose.position.x, arm.get_current_pose().pose.position.y, arm.get_current_pose().pose.position.z]))
                print("===================================================")

            elif key == 'g':
                status = status + 5
                gripper.set_joint_value_target([0.01, 0.01])
                gripper.go(wait = True)
                rospy.sleep(1)
                gripper.stop()
                gripper.clear_pose_targets()
                print("===================================================")
                print(vels(target_linear_vel,target_angular_vel))
                print(joints(arm.get_current_joint_values()))
                print(kinematics([arm.get_current_pose().pose.position.x, arm.get_current_pose().pose.position.y, arm.get_current_pose().pose.position.z]))
                print("===================================================")
            elif key == 'f':
                status = status + 5
                gripper.set_joint_value_target([0.003, 0.003])
                gripper.go(wait = True)
                rospy.sleep(1)
                gripper.stop()
                gripper.clear_pose_targets()
                print("===================================================")
                print(vels(target_linear_vel,target_angular_vel))
                print(joints(arm.get_current_joint_values()))
                print(kinematics([arm.get_current_pose().pose.position.x, arm.get_current_pose().pose.position.y, arm.get_current_pose().pose.position.z]))
                print("===================================================")
            else:
                if (key == '\x03') or (key == 'q'):
                    break

            if status == 20 :
                print(msg)
                status = 0

            twist = Twist()

            control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
            twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

            control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

            pub.publish(twist)

    except:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
