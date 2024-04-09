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
        self.traj=[]
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


    def set_motion_time(self, motion_time):
        """Set the estimated motion time for the robot arm
        Args:
            motion_time (float): The estimated motion time in seconds"""
        self.motion_time = motion_time

    def estimate_motion_time(self, start, target) -> float:
        """Estimate the motion time for the robot arm
        Args:
            start (list): The start grid position
            target (list): The target grid position
            Returns:
                float: The estimated motion time"""
        x_start, y_start, z_start = start[TASK_CONFIG.x], start[TASK_CONFIG.y], start[TASK_CONFIG.z]
        x_target, y_target, z_target = target[TASK_CONFIG.x], target[TASK_CONFIG.y], target[TASK_CONFIG.z]

        # if the x and y are the same, the motion time is 0.2
        if x_start == x_target and y_start == y_target:
            return 0.2

    def set_n_steps_motion(self, dt=0.05):
        """Set the number of steps for the motion
        Args:
            dt (float): The time step in seconds"""
        
        self.n_steps_motion = int(self.motion_time / dt)
    
    def plot_trajectory(self):
        """Plot the trajectory of the robot arm"""
        # self.traj.plot(block=True)
        self.plot(self.traj.q, block=True)

    def compute_trajectory(self, start, target):
        """Compute the trajectory of the robot arm
        Args:
            start (list): The start joint positions
            target (list): The target joint positions"""
        self.traj = rtb.jtraj(start, target, t=self.n_steps_motion)
        # self.plot_trajectory()

if __name__ == "__main__":
    import yaml

    # load task
    with open(f"../config/tasks/2_blocks.yaml", "r") as file:
        task_config = yaml.safe_load(file)

    ur = UR3e()
    final_traj = []
    final_time = []

    origin0 = task_config[0][TASK_CONFIG.ORIGIN]
    target0 = task_config[0][TASK_CONFIG.TARGET]
    origin1 = task_config[1][TASK_CONFIG.ORIGIN]
    target1 = task_config[1][TASK_CONFIG.TARGET]
    home = TASK_CONFIG.HOME_POSITION
    HOME = ur.compute_joint_positions_xy(home[TASK_CONFIG.x], home[TASK_CONFIG.y])
    BGP0, GP0, BTP0, TP0 = ur.compute_joint_positions_origin_target(origin0, target0)
    BGP1, GP1, BTP1, TP1 = ur.compute_joint_positions_origin_target(origin1, target1)

    # continue HOME position corresponding to 3s
    ur.set_motion_time(3)
    ur.set_n_steps_motion()
    time_vector = np.linspace(0, ur.motion_time, ur.n_steps_motion)
    final_time=np.append(final_time, time_vector, axis=0)
    final_traj=np.append(final_traj, np.tile(HOME, ur.n_steps_motion), axis=0)

    ur.set_motion_time(1.5)
    ur.set_n_steps_motion()

    ur.compute_trajectory(HOME, BGP0)
    final_traj=np.append(final_traj, ur.traj.q)
    final_time=np.append(final_time, np.linspace(final_time[-1]+0.05, ur.motion_time+final_time[-1], ur.n_steps_motion), axis=0)

    ur.set_motion_time(1.15)
    ur.set_n_steps_motion()
    
    ur.compute_trajectory(BGP0, GP0)
    final_traj=np.append(final_traj, ur.traj.q)
    final_time=np.append(final_time, np.linspace(final_time[-1]+0.05, ur.motion_time+final_time[-1], ur.n_steps_motion), axis=0)

    # continue last position corresponding to 0.7s  
    ur.set_motion_time(0.7)
    ur.set_n_steps_motion()
    time_vector = np.linspace(final_time[-1]+0.05, ur.motion_time+final_time[-1], ur.n_steps_motion)
    final_time=np.append(final_time, time_vector, axis=0)
    final_traj=np.append(final_traj, np.tile(ur.traj.q[-1], ur.n_steps_motion), axis=0)

    ur.set_motion_time(1.15)
    ur.set_n_steps_motion()

    ur.compute_trajectory(GP0, BGP0)
    final_traj=np.append(final_traj, ur.traj.q)
    final_time=np.append(final_time, np.linspace(final_time[-1]+0.05, ur.motion_time+final_time[-1], ur.n_steps_motion), axis=0)

    ur.set_motion_time(2.2)
    ur.set_n_steps_motion()

    ur.compute_trajectory(BGP0, BTP0)
    final_traj=np.append(final_traj, ur.traj.q)
    final_time=np.append(final_time, np.linspace(final_time[-1]+0.05, ur.motion_time+final_time[-1], ur.n_steps_motion), axis=0)

    ur.set_motion_time(0.8)
    ur.set_n_steps_motion()

    ur.compute_trajectory(BTP0, TP0)
    final_traj=np.append(final_traj, ur.traj.q)
    final_time=np.append(final_time, np.linspace(final_time[-1]+0.05, ur.motion_time+final_time[-1], ur.n_steps_motion), axis=0)

    # continue last position corresponding to 0.3s (partly release)
    ur.set_motion_time(0.3)
    ur.set_n_steps_motion()
    time_vector = np.linspace(final_time[-1]+0.05, ur.motion_time+final_time[-1], ur.n_steps_motion)
    final_time=np.append(final_time, time_vector, axis=0)
    final_traj=np.append(final_traj, np.tile(ur.traj.q[-1], ur.n_steps_motion), axis=0)

    ur.set_motion_time(0.8)
    ur.set_n_steps_motion()

    ur.compute_trajectory(TP0, BTP0)
    final_traj=np.append(final_traj, ur.traj.q)
    final_time=np.append(final_time, np.linspace(final_time[-1]+0.05, ur.motion_time+final_time[-1], ur.n_steps_motion), axis=0)

    # continue last position corresponding to 1.1s (release)
    ur.set_motion_time(1.1)
    ur.set_n_steps_motion()
    time_vector = np.linspace(final_time[-1]+0.05, ur.motion_time+final_time[-1], ur.n_steps_motion)
    final_time=np.append(final_time, time_vector, axis=0)
    final_traj=np.append(final_traj, np.tile(ur.traj.q[-1], ur.n_steps_motion), axis=0)

    ur.set_motion_time(2.2)
    ur.set_n_steps_motion()

    ur.compute_trajectory(BTP0, BGP1)
    final_traj=np.append(final_traj, ur.traj.q)
    final_time=np.append(final_time, np.linspace(final_time[-1]+0.05, ur.motion_time+final_time[-1], ur.n_steps_motion), axis=0)

    ur.set_motion_time(1.15)
    ur.set_n_steps_motion()

    ur.compute_trajectory(BGP1, GP1)
    final_traj=np.append(final_traj, ur.traj.q)
    final_time=np.append(final_time, np.linspace(final_time[-1]+0.05, ur.motion_time+final_time[-1], ur.n_steps_motion), axis=0)
    
    # continue last position corresponding to 0.7s
    ur.set_motion_time(0.7)
    ur.set_n_steps_motion()
    time_vector = np.linspace(final_time[-1]+0.05, ur.motion_time+final_time[-1], ur.n_steps_motion)
    final_time=np.append(final_time, time_vector, axis=0)
    final_traj=np.append(final_traj, np.tile(ur.traj.q[-1], ur.n_steps_motion), axis=0)

    ur.set_motion_time(1.15)
    ur.set_n_steps_motion()

    ur.compute_trajectory(GP1, BGP1)
    final_traj=np.append(final_traj, ur.traj.q)
    final_time=np.append(final_time, np.linspace(final_time[-1]+0.05, ur.motion_time+final_time[-1], ur.n_steps_motion), axis=0)
    
    ur.set_motion_time(2.2)
    ur.set_n_steps_motion()

    ur.compute_trajectory(BGP1, BTP1)
    final_traj=np.append(final_traj, ur.traj.q)
    final_time=np.append(final_time, np.linspace(final_time[-1]+0.05, ur.motion_time+final_time[-1], ur.n_steps_motion), axis=0)
    
    ur.set_motion_time(0.8)
    ur.set_n_steps_motion()

    ur.compute_trajectory(BTP1, TP1)
    final_traj=np.append(final_traj, ur.traj.q)
    final_time=np.append(final_time, np.linspace(final_time[-1]+0.05, ur.motion_time+final_time[-1], ur.n_steps_motion), axis=0)

    # continue last position corresponding to 0.3s (partly release)
    ur.set_motion_time(0.3)
    ur.set_n_steps_motion()
    time_vector = np.linspace(final_time[-1]+0.05, ur.motion_time+final_time[-1], ur.n_steps_motion)
    final_time=np.append(final_time, time_vector, axis=0)
    final_traj=np.append(final_traj, np.tile(ur.traj.q[-1], ur.n_steps_motion), axis=0)

    ur.set_motion_time(0.8)
    ur.set_n_steps_motion()

    ur.compute_trajectory(TP1, BTP1)
    final_traj=np.append(final_traj, ur.traj.q)
    final_time=np.append(final_time, np.linspace(final_time[-1]+0.05, ur.motion_time+final_time[-1], ur.n_steps_motion), axis=0)

    # continue last position corresponding to 1.1s (release)
    ur.set_motion_time(1.1)
    ur.set_n_steps_motion()
    time_vector = np.linspace(final_time[-1]+0.05, ur.motion_time+final_time[-1], ur.n_steps_motion)
    final_time=np.append(final_time, time_vector, axis=0)
    final_traj=np.append(final_traj, np.tile(ur.traj.q[-1], ur.n_steps_motion), axis=0)

    ur.set_motion_time(1.2)
    ur.set_n_steps_motion()

    ur.compute_trajectory(BTP1, HOME)
    final_traj=np.append(final_traj, ur.traj.q)
    final_time=np.append(final_time, np.linspace(final_time[-1]+0.05, ur.motion_time+final_time[-1], ur.n_steps_motion), axis=0)


    # for each block of 6 values in final_traj, split into new row
    final_traj = final_traj.reshape(-1, 6)

    # save trajectory to csv file
    import pandas as pd
    df = pd.DataFrame(final_traj, columns=['actual_q_0','actual_q_1','actual_q_2','actual_q_3','actual_q_4','actual_q_5'])
    # add time column
    df['timestamp'] = final_time
    df.to_csv("trajectory_dt.csv", sep=' ', index=False)