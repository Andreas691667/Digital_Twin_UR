# TODO: Insert description and mention santiago

import numpy as np
from spatialmath import SE3
from spatialmath.base import trnorm
import roboticstoolbox as rtb
from numpy import pi
from sys import path

path.append("..")
from config.task_config import TASK_CONFIG


# Changed named to UR3e
class UR3e(rtb.DHRobot):
    """Model of the Universal Robotics UR3e robot arm"""

    def __init__(self):
        link1 = rtb.RevoluteMDH(d=0.15185, a=0, alpha=0, qlim=[-3.82, -1])
        # qlim=[2.39, 5.06]
        # [-4.36, -1.13]
        # [-1, -3.82]
        link2 = rtb.RevoluteMDH(d=0, a=0, alpha=pi / 2)
        link3 = rtb.RevoluteMDH(d=0, a=-0.24355, alpha=0)  # changed a to negative
        link4 = rtb.RevoluteMDH(d=0.13105, a=-0.2132, alpha=0)  # changed a to negative
        link5 = rtb.RevoluteMDH(d=0.08535, a=0, alpha=pi / 2)
        link6 = rtb.RevoluteMDH(d=0.0921, a=0, alpha=-pi / 2)

        # Get parameters from config
        self.X_BASE_MIN = TASK_CONFIG.GRID_PARAMETERS[TASK_CONFIG.X_BASE_MIN]
        self.Y_BASE_MIN = TASK_CONFIG.GRID_PARAMETERS[TASK_CONFIG.Y_BASE_MIN]
        self.Z_BASE_MIN = TASK_CONFIG.GRID_PARAMETERS[TASK_CONFIG.Z_BASE_MIN]
        self.HOLE_DIST = TASK_CONFIG.GRID_PARAMETERS[TASK_CONFIG.HOLE_DIST]

        # IK params
        self.q0_ = [-2.64, -1.27, -1.76, -2.06, -1.57, 2.1]

        super().__init__([link1, link2, link3, link4, link5, link6], name="UR3e")

    def __compute_base_coordinates(self, x_g, y_g, z_g, rx=0, ry=0, rz=0):
        """Compute the x, y, z position of the flexcell based on the hole number"""
        # Calculate base values
        comp_x = self.X_BASE_MIN + x_g * self.HOLE_DIST
        comp_y = self.Y_BASE_MIN + y_g * self.HOLE_DIST
        comp_z = self.Z_BASE_MIN + z_g * self.HOLE_DIST

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

        q0_ = [(3 * pi) / 2, -0.66, 0.76, -pi / 2, -pi / 2, 2.35]
        # sol1 = self.ikine_LM(T, q0=[0, -np.pi / 2, 0, -np.pi / 2, 0, 0])
        # sol1 = self.ikine_LM(T, q0=q0_, joint_limits=True)
        sol1 = self.ikine_LM(T, q0=q0_, joint_limits=False)

        if sol1.success:
            solution1 = sol1.q
            if rounded:
                solution1 = np.round(solution1, 2)

            # Update guess
            # self.q0_ = solution1

            return solution1
        return np.nan

    def compute_joint_positions_xy(self, x_g, y_g, grip_pos: bool = False):
        """Compute the joint positions for the UR3e robot arm given x and y of the grid"""
        GRIP_Z = TASK_CONFIG.GRID_COORDINATES[TASK_CONFIG.GRIP_Z]
        BEFORE_GRIP_Z = TASK_CONFIG.GRID_COORDINATES[TASK_CONFIG.BEFORE_GRIP_Z]
        z_g = GRIP_Z if grip_pos else BEFORE_GRIP_Z
        x, y, z = self.__compute_base_coordinates(x_g, y_g, z_g)
        q = self.__compute_ik_num(x, y, z, rounded=False)
        q_valid = self.check_joint_validity(q)
        return q_valid

    def compute_joint_positions_origin_target(self, origin, target):
        """Compute the joint positions for the UR3e robot arm given origin and target"""
        origin_q_start = self.compute_joint_positions_xy(
            origin[TASK_CONFIG.x], origin[TASK_CONFIG.y]
        )
        origin_q = self.compute_joint_positions_xy(
            origin[TASK_CONFIG.x], origin[TASK_CONFIG.y], grip_pos=True
        )
        target_q_start = self.compute_joint_positions_xy(
            target[TASK_CONFIG.x], target[TASK_CONFIG.y]
        )
        target_q = self.compute_joint_positions_xy(
            target[TASK_CONFIG.x], target[TASK_CONFIG.y], grip_pos=True
        )

        if origin[TASK_CONFIG.ROTATE_WRIST]:
            origin_q_start[-1] -= pi/2
            origin_q[-1] -= pi/2
        if target[TASK_CONFIG.ROTATE_WRIST]:
            target_q_start[-1] -= pi/2
            target_q[-1] -= pi/2

        return origin_q_start, origin_q, target_q_start, target_q

    def check_joint_validity(self, q):
        """Check if the joint positions are valid"""
        # get base position
        if q[0] < 0:
            q[0] += 2 * pi

        if q[-1] < 0:
            q[-1] += 2 * pi

        return q


# if __name__ == "__main__":
#     ur = UR3e()
#     q = ur.compute_joint_positions(-3,5, grip_pos=True)
#     q = ur.check_joint_validity(q)
#     print(q)
#     # print in degrees
#     print(np.rad2deg(q))
