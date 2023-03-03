from nuro_arm import RobotArm

# connect to robot
controller_type = 'real'  # or 'sim'
robot = RobotArm(controller_type)

# turn off motors, useful for guiding robot by hand

# turn on motors
robot.active_mode()

# jpos = [0.3, 0, 0, 0, 0]

# sends command, returns once motion stops
# robot.move_arm_jpos(jpos)
print('started')
# see achieved joint angles
# achieved_jpos = robot.get_arm_jpos()
#
# ee_pos = [1, 1,1 ]

# sends command, returns once motion stops
robot.open_gripper()

# robot.move_hand_to(ee_pos)

# see achieved end effector position
# achieved_ee_pos, _ = robot.get_hand_pose()
robot.close_gripper()

# sends command, returns once motion stops
# robot.move_arm_jpos(jpos)
