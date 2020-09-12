#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

rospy.init_node('move_arm')
pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)

jt = JointTrajectory()
jt.header.frame_id = "base_link"
jt.joint_names = ['hip', 'shoulder', 'elbow', 'wrist']
points = JointTrajectoryPoint()
jt.points.append(points)
jt.points[0].positions = [0.0, 0.0, 0.0, 0.0]
jt.points[0].velocities = [3.0, 1.0, 1.0, 1.0]

rate = rospy.Rate(1)
while not rospy.is_shutdown():
	answer = input('Do you want to exit? Enter y if yes or any other key : ')
	if answer[0] == 'y':
		break
	print()
	for i in range(4):
		pos = float(input(jt.joint_names[i] + " position : "))
		jt.points[0].positions[i] = pos 
	print()
	jt.points[0].time_from_start = rospy.Duration(1)
	pub.publish(jt)
	rate.sleep()
