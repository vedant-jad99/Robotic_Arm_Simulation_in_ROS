#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

rospy.init_node('move_arm')
pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
joint_dict = {'hip': 0, 'shoulder': 1, 'elbow': 2, 'wrist': 3}

jt = JointTrajectory()
jt.header.frame_id = "base_link"
jt.joint_names = ['hip', 'shoulder', 'elbow', 'wrist', 'l_g_base', 'r_g_base']
points = JointTrajectoryPoint()
jt.points.append(points)
initial_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
jt.points[0].positions[:] = initial_positions[:]
gripper_extend = True

print("The Robotic Arm is online. The following are valid commands: \n1. Move\n2. Stop\n3. Release \n4. Close\n5. Change\n")
print("If command == move, input the values of joint positions. Joints - hip, shoulder, elbow, wrist.")
print("Relase command is to release the gripper.\nClose command is to close the gripper.\nChange command is to change the position of only one joint, keeping the others same.")
print("eg. Change\nhip : 0.45 (Only this format is accepted)\n")

rate = rospy.Rate(1)
while not rospy.is_shutdown():
	command = input("Enter : ")
	command = command.lower()
	if "stop" in command:
		jt.points[0].positions[:] = initial_positions[:]
	elif "move" in command:
		for i in range(4):
			pos = float(input(jt.joint_names[i] + " position : "))
			jt.points[0].positions[i] = pos
	elif "release" in command:
		if gripper_extend == True:
			print("Already in release state!")
		else:
			jt.points[0].positions[4] = 0
			jt.points[0].positions[5] = 0
			gripper_extend = True
	elif "close" in command:
		if gripper_extend == False:
			print("Already closed!")
		else:
			jt.points[0].positions[4] = 0.5236
			jt.points[0].positions[5] = -0.5236
			gripper_extend = False
	elif "change" in command:
		next_line = input()
		broke_command = next_line.split(" ")
		if ":" in broke_command:
			try:
				joint_index = joint_dict[broke_command[0].lower()]
				jt.points[0].positions[joint_index] = float(broke_command[2])
			except:
				print("Joint not found. Have you entered it correctly?\n")
		else:
			print("Not the right format.\n")
	else:
		print("Unrecognized command! Try again.\n")
	print()
	jt.points[0].time_from_start = rospy.Duration(1)
	pub.publish(jt)
	rate.sleep()
	if "stop" in command:
		break
