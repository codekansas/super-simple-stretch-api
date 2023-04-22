from __future__ import print_function

import math

from stretch.prismatic_joint import PrismaticJoint


class Arm(PrismaticJoint):
    def __init__(self, usb: str | None = None) -> None:
        super().__init__(name="arm", usb=usb)

    def motor_rad_to_translate_m(self, ang):  # input in rad, output m
        return (
            self.params["chain_pitch"] * self.params["chain_sprocket_teeth"] / self.params["gr_spur"] / (math.pi * 2)
        ) * ang

    def translate_m_to_motor_rad(self, x):
        return x / (
            self.params["chain_pitch"] * self.params["chain_sprocket_teeth"] / self.params["gr_spur"] / (math.pi * 2)
        )

    def home(self, end_pos=0.1, to_positive_stop=False, measuring=False):
        return PrismaticJoint.home(self, end_pos=end_pos, to_positive_stop=to_positive_stop, measuring=measuring)
