import numpy as np
from spatialmath import SE3
from spatialmath.base import trnorm
import roboticstoolbox as rtb
from numpy import pi


# Changed named to UR3e
class UR3e(rtb.DHRobot):
    """Model of the Universal Robotics UR3e robot arm"""

    def __init__(self):
        link1 = rtb.RevoluteMDH(d=0.15185, a=0, alpha=0)
        link2 = rtb.RevoluteMDH(d=0, a=0, alpha=pi / 2)
        link3 = rtb.RevoluteMDH(d=0, a=-0.24355, alpha=0)  # changed a to negative
        link4 = rtb.RevoluteMDH(d=0.13105, a=-0.2132, alpha=0)  # changed a to negative
        link5 = rtb.RevoluteMDH(d=0.08535, a=0, alpha=pi / 2)
        link6 = rtb.RevoluteMDH(d=0.0921, a=0, alpha=-pi / 2)
        super().__init__([link1, link2, link3, link4, link5, link6], name="UR3e")


class UR3e_RL(UR3e):
    """Model of the Universal Robotics UR3e robot arm with additional methods"""

    def __compute_xyz_flexcell(self, X, Y, grip_pos: bool, Z=0, rx=0, ry=0, rz=0):
        """Compute the x, y, z position of the flexcell based on the hole number"""

        # if grip_pos is True, the robot is in the grip position
        # else it should be higher from the block
        if grip_pos:
            Z_table_level = 0.185
        else:
            Z_table_level = 0.235

        # TODO: Update these and put them in config
        YMIN = 0
        YMAX = 4
        XMIN = 0
        XMAX = 8
        HOLE_DIST = 0.04

        # TODO: Update these and put them in config (*_max not used yet)
        x_max = 0.152
        x_min = -0.12051  # 0.45
        y_max = 0.4729
        y_min = 0.28071

        comp_x = x_min + ((X) * HOLE_DIST)
        comp_y = y_min + ((Y) * HOLE_DIST)
        comp_z = Z_table_level + Z * HOLE_DIST

        return comp_x, comp_y, comp_z

    def __compute_ik_num(self, x, y, z, rounded=False):
        """Compute the inverse kinematics for the UR3e robot arm"""
        T = SE3(
            trnorm(
                np.array(
                    [
                        [np.cos(-np.pi), 0, np.sin(-np.pi), x],
                        [0, 1, 0, y],
                        [np.sin(-np.pi), 0, np.cos(-np.pi), z],
                        [0, 0, 0, 1],
                    ]
                )
            )
        )  # Rotation of pi around the y-axis

        # initial guess vector 
        q0_ = [-pi / 2, -pi / 4, pi / 4, -pi / 2, -pi / 2, pi]
        # q0_ = [-2.33, -0.66, 0.76, -pi/2, -pi/2, 2.35]
        # sol1 = self.ikine_LM(T, q0=[0, -np.pi / 2, 0, -np.pi / 2, 0, 0])
        sol1 = self.ikine_LM(T, q0=q0_)

        if sol1.success:
            solution1 = sol1.q
            if rounded:
                solution1 = np.round(solution1, 2)
            return solution1
        return np.nan

    def compute_joint_positions(self, x, y, grip_pos: bool = False):
        """Compute the joint positions for the UR3e robot arm"""
        x, y, z = self.__compute_xyz_flexcell(x, y, grip_pos)
        return self.__compute_ik_num(x, y, z, rounded=False)
