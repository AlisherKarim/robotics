import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


def main():
	moveit_commander.roscpp_initialize(sys.argv) #moveit itself init
	robot = moveit_commander.RobotCommander() # robot’s kinematic model and the robot’s current joint states
	scene = moveit_commander.PlanningSceneInterface() #remote interface for getting, setting, and updating the robot’s internal understanding of the surrounding world
	arm = moveit_commander.MoveGroupCommander('arm') # name of movegroup 1 is “arm”
	gripper = moveit_commander.MoveGroupCommander('gripper') #name of movegroup 2 is “gripper”
	arm.set_planning_time(2) # Thats the value TA likes to use


	joint_values = arm.get_current_joint_values() #How to get joint states
	joint_values[0] = 0
	joint_values[1] = 0
	joint_values[2] = 0
	joint_values[3] = 0

	print("Starting...")


	arm.go(joints = joint_values, wait = True)

	print("Execution finished")

	rospy.sleep(5)

	arm.stop()
	arm.clear_pose_targets()






if __name__ == "__main__":
    main()