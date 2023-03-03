import time

import numpy as np
import xarm

# arm2 = xarm.Controller('USB4991477B3033', debug=True)
#

import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D

# ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')

from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink

from movement import forward


def convert_angle(angle):
    return (angle) * (180 / np.pi)


def convert_chain_1(angle_set):
    angle1 = convert_angle(angle_set[0])
    angle2 = convert_angle(-angle_set[1])
    angle3 = convert_angle(-angle_set[2])
    angle4 = convert_angle(angle_set[3])
    angle5 = convert_angle(angle_set[4])
    angle6 = convert_angle(angle_set[5])

    return [
        [3, angle4],
        [4, angle3],
        [5, angle2],
        [6, angle1]
    ]





class XArmControl:
    def __init__(self, usb_serial_number, base_position=None):
        if base_position is None:
            base_position = [-7, 10, 20]

        self.base_position = base_position
        self.arm = xarm.Controller(usb_serial_number, debug=True)
        # self.arm = xarm.Controller('USB4996587C3033', debug=True)

        base_chain = [
            URDFLink(
                name="base",
                origin_translation=np.asarray(base_position),
                origin_orientation=np.asarray([0, 0, 0]),
                rotation=np.asarray([0, 0, 1]),
                bounds=(-np.pi / 2, np.pi / 2),
            ),
            URDFLink(
                name="base",
                origin_translation=np.asarray([0, 0, 2]),
                origin_orientation=np.asarray([0, 0, 0]),
                rotation=np.asarray([0, 1, 0]),
                bounds=(-np.pi / 2, np.pi / 2),

            ),
            URDFLink(
                name="shoulder",
                origin_translation=np.asarray([0, 0, 10]),
                origin_orientation=np.asarray([0, 0, 0]),
                rotation=np.asarray([0, 1, 0]),
                bounds=(-np.pi / 2, np.pi / 2),
            ),
            URDFLink(
                name="elbow",
                origin_translation=np.asarray([0, 0, 10]),
                origin_orientation=np.asarray([0, 0, 0]),
                rotation=np.asarray([0, 1, 0]),
                bounds=(-np.pi / 2, np.pi / 2),
            ),
            URDFLink(
                name="wrist",
                origin_translation=np.asarray([0, 0, 5]),
                origin_orientation=np.asarray([0, 0, 0]),
                rotation=np.asarray([0, 0, 1]),
                bounds=(-np.pi / 2, np.pi / 2),
            )]
        self.chain_closed = Chain(name='left_arm', links=[
            *base_chain,
            URDFLink(
                name="grabber",
                origin_translation=np.asarray([0, 0, 13]),
                origin_orientation=np.asarray([0, 0, 0]),
                joint_type="fixed",
                bounds=(-np.pi / 2, np.pi / 2),
            )
        ])
        self.chain_opened = Chain(name='left_arm', links=[
            *base_chain,
            URDFLink(
                name="grabber",
                origin_translation=np.asarray([0, 0, 8]),
                origin_orientation=np.asarray([0, 0, 0]),
                joint_type="fixed",
                bounds=(-np.pi / 2, np.pi / 2),
            )
        ])
        self.chain = self.chain_closed
        self.opened = False
        self.ik = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def close_chain(self, position):
        x = position[0] - self.base_position[0]
        y = position[1] - self.base_position[1]
        theta = np.arctan(y / x) * 180 / np.pi + 90
        theta2 = 700
        if theta > 125:
            theta = 125.0
        if theta< -125:
            theta =  -125.0
        return [
            # [1, theta2],
            [2, theta]
        ]

    def open_chain(self,position):
        x = position[0] - self.base_position[0]
        y = position[1] - self.base_position[1]
        theta = np.arctan(y / x) * 180 / np.pi + 90
        if theta > 125:
            theta = 125
        if theta < -125:
            theta = -125
        theta2 = 0
        return [
            # [1, theta2],
            [2, theta]
        ]
    def go_to_location(self, x, y, z):
        self.ik = self.chain.inverse_kinematics(np.asarray([x, y, z]))
        a = convert_chain_1(self.ik)
        if self.opened:
            self.arm.setPosition(a)
        else:
            print(self.close_chain([x, y, z]))
            self.arm.setPosition([*a, *self.close_chain([x, y, z])])

    def open_arm(self):
        self.arm.openGripper(0)
        self.chain = self.chain_opened
        time.sleep(1)
        self.opened = True

    def pick_up_object(self, x, y, z):
        self.ik = self.chain.inverse_kinematics(np.asarray([x, y, z + 10]))
        a = convert_chain_1(self.ik)
        b = self.open_chain([x, y, 0])
        self.arm.setPosition([*b, *a])
        self.arm.openGripper(0)
        self.chain = self.chain_opened
        self.opened = True
        self.ik = self.chain.inverse_kinematics(np.asarray([x, y, z]), target_orientation=np.asarray([0, 0, 1]),
                                                orientation_mode='X', initial_position=self.ik)

        a = convert_chain_1(self.ik)
        self.arm.setPosition(a)
        self.chain = self.chain_closed
        self.opened = False
        self.ik = self.chain.inverse_kinematics(np.asarray([x, y, z]), target_orientation=np.asarray([0, 0, 1]),
                                                orientation_mode='X', initial_position=self.ik)

        b = self.close_chain([x, y, 0])
        self.arm.closeGripper(1000)
        self.arm.setPosition([*b, *a])
        time.sleep(1)
        b = self.close_chain([x, y, 0])
        self.arm.setPosition([*b])
        time.sleep(1)


# forward(15)
#
# arm = XArmControl('USB4996587C3033')
# arm1 = XArmControl('USB4991477B3033', [-7, 35, 18.5])
#
# # arm.pick_up_object(9, 15, 3)
# # arm1.pick_up_object(9, 15, 3)
# # arm.go_to_location(20, 15, 5)
# # arm1.go_to_location(20, 15, 5)
#
# arm.go_to_location(10, 20, 20)
# arm1.go_to_location(10, 25, 20)
# arm.arm.closeGripper(1000)
# arm1.arm.closeGripper(1000)
# # print("take it forward")
# time.sleep(1)
# arm.arm.openGripper(0)
# arm1.arm.openGripper(0)
# time.sleep(2)
# arm.go_to_location(10, 17, 10)
# arm1.go_to_location(10, 28, 10)
# time.sleep(2)
#
# arm.go_to_location(10, 17, 5)
# arm1.go_to_location(10, 28, 5)
# time.sleep(2)
# arm.arm.closeGripper(1000)
# arm1.arm.closeGripper(1000)
# time.sleep(2)
#
# arm.go_to_location(10, 17, 7)
# arm1.go_to_location(10, 28, 7)
# time.sleep(2)
#
# arm.go_to_location(3, 17, 7)
# arm1.go_to_location(3, 28, 7)
# time.sleep(2)
#
# arm.go_to_location(3, 17, 10)
# arm1.go_to_location(3, 28, 10)
# time.sleep(2)
#
#
# arm.go_to_location(20, 17, 10)
# arm1.go_to_location(20, 28, 10)
# time.sleep(2)
#
# arm.go_to_location(20, 17, 5)
# arm1.go_to_location(20, 28, 5)
# time.sleep(2)
#
# arm.arm.openGripper(0)
# arm1.arm.openGripper(0)
# time.sleep(2)
#
# arm.go_to_location(20, 17, 15)
# arm1.go_to_location(20, 28, 15)
# time.sleep(2)
#
# time.sleep(1)
# arm.go_to_location(9, 15, 40)
# arm1.go_to_location(9, 15, 40)
# time.sleep(1)
# arm.open_arm()
# arm1.open_arm()
# arm.go_to_location(0, 0, 0)
# arm1.go_to_location(0, 0, 0)
