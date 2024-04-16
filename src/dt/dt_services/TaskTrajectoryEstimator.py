import numpy as np
import pandas as pd
import datetime
from TrajectoryTimingEstimator import TrajectoryTimingEstimator


class TaskTrajectoryEstimator:
    """Class to estimate the trajectory of a task."""

    def __init__(self, model) -> None:
        self.robot_model = model
        self.traj_q = []
        self.traj_qd = []
        self.traj_qdd = []
        self.time = []

    def __save_traj_to_file(self, file_name):
        """Save the trajectory to a file specified by file name."""
        dfq = pd.DataFrame(
            self.traj_q,
            columns=[
                "actual_q_0",
                "actual_q_1",
                "actual_q_2",
                "actual_q_3",
                "actual_q_4",
                "actual_q_5",
            ],
        )
        dfqd = pd.DataFrame(
            self.traj_qd,
            columns=[
                "actual_qd_0",
                "actual_qd_1",
                "actual_qd_2",
                "actual_qd_3",
                "actual_qd_4",
                "actual_qd_5",
            ],
        )

        dfqdd = pd.DataFrame(
            self.traj_qdd,
            columns=[
                "actual_qdd_0",
                "actual_qdd_1",
                "actual_qdd_2",
                "actual_qdd_3",
                "actual_qdd_4",
                "actual_qdd_5",
            ],
        )

        dft = pd.DataFrame(self.time, columns=["timestamp"])

        df = pd.concat([dfq, dfqd, dfqdd, dft], axis=1)

        file_name = (
            file_name
            if file_name is not None
            else datetime.datetime.now().strftime("%m_%d_%Y_%H_%M_%S")
        )

        df.to_csv(f"../dt_trajectories/{file_name}.csv", sep=" ", index=False)

    def estimate_trajectory(
        self, task_with_timings, start_time = 0, save_to_file=False, file_name=None
    ):
        """Estimate the trajectory of a task.
        params:
            :param Mx13 ndarray task_with_timings: Array with all starts and ends of the task and the timing intervals.
                                                   Mx13 array where M=N+D, N is the number of start-target-waypoints and D is the number delays
                                                   For a row r, r[0:6] is the start, r[6:12] is the target and r[12] is the motion time.
            :param float start_time: The start time of the trajectory.
            :param bool save_to_file: If True, save the trajectory to a file.
            :param str file_name: The name of the file to save the trajectory.

        :return: Estimated trajectory.
        :rtype: ndarray"""

        # get first start in task_with_timings that is not None
        start = None
        for elem in task_with_timings:
            if (elem[0:6] != [16] * 6).all():
                start = elem[0:6]
                break

        # initialize the trajectory vectors
        final_traj_q = []
        final_traj_qd = []
        final_traj_qdd = []
        final_time = [start_time]

        last_traj_q = [start]
        last_traj_qd = [0] * 6
        last_traj_qdd = [0] * 6

        for elem in task_with_timings:
            # Get the start, target and motion time
            start = None if (elem[0:6] == [16] * 6).all() else elem[0:6]
            target = None if (elem[6:12] == [16] * 6).all() else elem[6:12]
            motion_time = elem[12]

            # Calculate the trajectory
            self.robot_model.set_motion_time(motion_time)
            n_steps = self.robot_model.set_n_steps_motion()

            # check that start and target is not None
            if start is not None and target is not None:
                # Compute the trajectory
                traj = self.robot_model.compute_trajectory(start, target)

                final_traj_q = np.append(final_traj_q, traj.q)
                final_traj_qd = np.append(final_traj_qd, traj.qd)
                final_traj_qdd = np.append(final_traj_qdd, traj.qdd)

                # Compute the time
                time_vector = np.linspace(
                    final_time[-1] + 0.05, motion_time + final_time[-1], n_steps
                )
                final_time = np.append(final_time, time_vector, axis=0)
                last_traj_q = traj.q
                last_traj_qd = traj.qd
                last_traj_qdd = traj.qdd

            # If there is a delay
            elif start is None and target is None:
                final_traj_q = np.append(
                    final_traj_q, np.tile(last_traj_q[-1], n_steps), axis=0
                )

                final_traj_qd = np.append(
                    final_traj_qd, np.tile(last_traj_qd[-1], n_steps), axis=0
                )

                final_traj_qdd = np.append(
                    final_traj_qdd, np.tile(last_traj_qdd[-1], n_steps), axis=0
                )

                time_vector = np.linspace(
                    final_time[-1] + 0.05, motion_time + final_time[-1], n_steps
                )
                final_time = np.append(final_time, time_vector, axis=0)

        # Reshape the final trajectory
        # pad with zeros to make the length of the trajectories multiple of 6
        if len(final_traj_q) % 6 != 0:
            final_traj_q = np.append(final_traj_q, np.zeros((6-len(final_traj_q)%6)))
        if len(final_traj_qd) % 6 != 0:
            final_traj_qd = np.append(final_traj_qd, np.zeros((6-len(final_traj_qd)%6)))
        if len(final_traj_qdd) % 6 != 0:
            final_traj_qdd = np.append(final_traj_qdd, np.zeros((6-len(final_traj_qdd)%6)))


        print(final_traj_q.shape, final_traj_qd.shape, final_traj_qdd.shape, len(final_time))

        self.traj_q = np.reshape(final_traj_q, (-1, 6))
        self.traj_qd = np.reshape(final_traj_qd, (-1, 6))
        self.traj_qdd = np.reshape(final_traj_qdd, (-1, 6))
        self.time = final_time

        # Save the trajectory to a file
        if save_to_file:
            self.__save_traj_to_file(file_name)

        return self.traj_q, self.traj_qd, self.traj_qdd, self.time


# Example of usage
# main
if __name__ == "__main__":
    import sys
    import yaml

    sys.path.append("../..")
    from ur3e.ur3e import UR3e

    with open(f"../../config/tasks/2_blocks.yaml", "r") as file:
        task_config = yaml.safe_load(file)

    model = UR3e()
    task_estimator = TaskTrajectoryEstimator(model)
    traj_timing_est = TrajectoryTimingEstimator(model)

    origin0 = task_config[0]["ORIGIN"]
    target0 = task_config[0]["TARGET"]
    origin1 = task_config[1]["ORIGIN"]
    target1 = task_config[1]["TARGET"]
    HOME = model.compute_joint_positions_xy(11, -2)
    BGP0, GP0, BTP0, TP0 = model.compute_joint_positions_origin_target(origin0, target0)
    BGP1, GP1, BTP1, TP1 = model.compute_joint_positions_origin_target(origin1, target1)
    BTP1[-1] -= np.pi/2
    TP1[-1] -= np.pi/2
    v_none = [16] * 6

    # Create a task with timings
    task_with_timings = [
        np.concatenate((v_none, v_none, [.6])),
        np.concatenate((HOME, BGP0, [1.3])),
        np.concatenate((BGP0, GP0, [0.8])),
        np.concatenate((v_none, v_none, [0.8])),
        np.concatenate((GP0, BGP0, [0.8])),
        np.concatenate((BGP0, BTP0, [2.4])),
        np.concatenate((BTP0, TP0, [0.8])),
        np.concatenate((v_none, v_none, [0.6])),
        np.concatenate((TP0, BTP0, [0.8])),
        np.concatenate((v_none, v_none, [1.5])),
        np.concatenate((BTP0, BGP1, [2.5])),
        np.concatenate((BGP1, GP1, [.8])),
        np.concatenate((v_none, v_none, [0.8])),
        np.concatenate((GP1, BGP1, [0.8])),
        np.concatenate((BGP1, BTP1, [3.7])),
        np.concatenate((BTP1, TP1, [0.8])),
        np.concatenate((v_none, v_none, [0.6])),
        np.concatenate((TP1, BTP1, [1.0])),
        np.concatenate((v_none, v_none, [.8])),
        np.concatenate((BTP1, HOME, [2.3])),
    ]

    task_with_timings2 = traj_timing_est.get_traj_timings(task_config)

    trajq, trajqd, trajqdd, time = task_estimator.estimate_trajectory(
        task_with_timings2,start_time=100, save_to_file=True, file_name="dt_traj_2_blocks"
    )

    model.plot_trajectory()
    
