#!/usr/bin/env python3
# A very basic TurtleBot script that moves TurtleBot forward indefinitely. Press CTRL + C to stop.  To run:
# On TurtleBot:
# roslaunch turtlebot_bringup minimal.launch
# On work station:
# python move.py

import rospy
from geometry_msgs.msg import Twist


class GoForward:
	def forward(self, lin, ang):
		rospy.loginfo("Moving - lin : {} ang : {}".format(lin, ang))
		# Twist is a datatype for velocity
		move_cmd = Twist()
		# let's go forward
		move_cmd.linear.x = lin
		# let's turns
		move_cmd.angular.z = ang
		# publish the velocity
		self.cmd_vel.publish(move_cmd)

	def __init__(self):
		# initiliaze
		rospy.init_node('GoForward', anonymous=False)

		# tell user how to stop TurtleBot
		rospy.loginfo("To stop TurtleBot CTRL + C")
		# What function to call when you ctrl + c
		rospy.on_shutdown(self.shutdown)
		# Create a publisher which can "talk" to TurtleBot and tell it to move
		# Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if you're not using TurtleBot2
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

		# TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
		r = rospy.Rate(10);

		angle = 50
		speed = -10
		PI = 3.1415926535897

		# as long as you haven't ctrl + c keeping doing...
		while not rospy.is_shutdown():
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
				# r.sleep()

			print("[!] FINISH ROTATE")
			self.target_angular_vel = 0
			self.publish_twist()
			rospy.sleep(5)
			break
		LIN_VEL_STEP_SIZE = 0.1

		meters = 1

		while not rospy.is_shutdown():
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
			break


	def publish_twist(self):
		twist = Twist()
		twist.linear.x = self.target_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
		twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = self.target_angular_vel
		self.pub.publish(twist)

	def shutdown(self):
        # stop turtlebot
		rospy.loginfo("Stop TurtleBot")
		# a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
		self.cmd_vel.publish(Twist())
		# sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
		rospy.sleep(1)


if __name__ == '__main__':
	try:
		move = GoForward()
	except:
 	    rospy.loginfo("GoForward node terminated.")