import numpy as np
import pandas as pd
import datetime

class TaskTrajectoryEstimator:
    """Class to estimate the trajectory of a task."""

    def __init__(self, model) -> None:
        self.robot_model = model
        self.traj_q = []
        self.traj_qd = []
        self.traj_qdd = []
        self.time = []
        self.dt = 0.05

    def save_traj_to_file(self, file_name, trajectory_q=None, trajectory_qd=None, trajectory_qdd=None, time=None):
        """Save the trajectory to a file specified by file name."""
        trajectory_q = trajectory_q if trajectory_q is not None else self.traj_q
        trajectory_qd = trajectory_qd if trajectory_qd is not None else self.traj_qd
        trajectory_qdd = trajectory_qdd if trajectory_qdd is not None else self.traj_qdd
        time = time if time is not None else self.time

        dfq = pd.DataFrame(
            trajectory_q,
            columns=[
                "actual_q_0",
                "actual_q_1",
                "actual_q_2",
                "actual_q_3",
                "actual_q_4",
                "actual_q_5",
            ],
        )
        # dfqd = pd.DataFrame(
        #     trajectory_qd,
        #     columns=[
        #         "actual_qd_0",
        #         "actual_qd_1",
        #         "actual_qd_2",
        #         "actual_qd_3",
        #         "actual_qd_4",
        #         "actual_qd_5",
        #     ],
        # )

        # dfqdd = pd.DataFrame(
        #     trajectory_qdd,
        #     columns=[
        #         "actual_qdd_0",
        #         "actual_qdd_1",
        #         "actual_qdd_2",
        #         "actual_qdd_3",
        #         "actual_qdd_4",
        #         "actual_qdd_5",
        #     ],
        # )

        dft = pd.DataFrame(time, columns=["timestamp"])

        # df = pd.concat([dfq, dfqd, dfqdd, dft], axis=1)
        df = pd.concat([dfq, dft], axis=1)

        # if file already exists in folder dt_trajectories, append a number to the file name
        file_name = (
            file_name
            if file_name is not None
            else datetime.datetime.now().strftime("%m_%d_%Y_%H_%M_%S")
        )

        # import os
        # i = 1
        # while os.path.exists(f"dt_trajectories/{file_name}"):
        #     file_name = str(i) + "_" + file_name
        #     i += 1

        df.to_csv(f"dt_trajectories/{file_name}", sep=" ", index=False)

    def update_time_vector(self, start_time, time_vec = None, index = 0):
        """Update the time vector by making start_time first element and then add dt."""
        # time_vec = time_vec if time_vec is not None else self.time
        # new_time = [start_time]
        # for i in range(1, len(time_vec)):
        #     new_time.append(new_time[i - 1] + 0.05)
        # self.time = new_time
        # return self.time
        time_vec[index] = start_time
        for i in range(index + 1, len(time_vec)):
            time_vec[i] = time_vec[i - 1] + self.dt
        self.time = time_vec
        return time_vec

    def estimate_trajectory(
        self, task_with_timings, start_time = 0, save_to_file=False, file_name=None
    ):
        """Estimate the trajectory of a task.
        params:
            :param Mx14 ndarray task_with_timings: Array with all starts and ends of the task and the timing intervals.
                                                   Mx13 array where M=N+D, N is the number of start-target-waypoints and D is the number delays
                                                   For a row r, r[0:6] is the start, r[6:12] is the target, r[12] is the motion time, r[13] are the descriptions for each move.
            :param float start_time: The start time of the trajectory.
            :param bool save_to_file: If True, save the trajectory to a file.
            :param str file_name: The name of the file to save the trajectory.

        :return: Estimated trajectory.
        :rtype: ndarray"""

        # get first start in task_with_timings that is not None
        start = None
        for elem in task_with_timings:
            if all(elem[0:6]):
                start = elem[0:6]
                break

        # initialize the trajectory vectors
        final_traj_q = []
        final_traj_qd = []
        final_traj_qdd = []
        final_traj_des = []
        final_time = []

        last_traj_q = [start]
        last_traj_qd = [0] * 6
        last_traj_qdd = [0] * 6

        for elem in task_with_timings:
            # Get the start, target and motion time and description
            start = None if not all(elem[0:6]) else elem[0:6]
            target = None if not all(elem[6:12]) else elem[6:12]
            motion_time = elem[12]
            description = elem[13]

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
                if len(final_time) == 0:
                    time_vector = np.linspace(start_time, motion_time, n_steps)
                else:
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

                # Compute the time
                if len(final_time) == 0:
                    time_vector = np.linspace(start_time, motion_time, n_steps)
                else:
                    time_vector = np.linspace(
                        final_time[-1] + 0.05, motion_time + final_time[-1], n_steps
                    )

                final_time = np.append(final_time, time_vector, axis=0)

            # update the description vector. Should repeat the description for each time step
            final_traj_des = np.append(final_traj_des, np.tile(description, n_steps))

        # Reshape the final trajectory
        # pad with zeros to make the length of the trajectories multiple of 6
        if len(final_traj_q) % 6 != 0:
            final_traj_q = np.append(final_traj_q, np.zeros((6-len(final_traj_q)%6)))
        if len(final_traj_qd) % 6 != 0:
            final_traj_qd = np.append(final_traj_qd, np.zeros((6-len(final_traj_qd)%6)))
        if len(final_traj_qdd) % 6 != 0:
            final_traj_qdd = np.append(final_traj_qdd, np.zeros((6-len(final_traj_qdd)%6)))

        self.traj_q = np.reshape(final_traj_q, (-1, 6))
        self.traj_qd = np.reshape(final_traj_qd, (-1, 6))
        self.traj_qdd = np.reshape(final_traj_qdd, (-1, 6))
        self.time = final_time

        # Save the trajectory to a file
        if save_to_file:
            self.save_traj_to_file(file_name)

        return self.traj_q, self.traj_qd, self.traj_qdd, self.time, final_traj_des


# Example of usage
# main
if __name__ == "__main__":
    import sys
    import yaml

    sys.path.append("../..")
    from models.robot_model.ur3e import UR3e
    from models.timing_model.TimingModel import TimingModel

    with open(f"../../config/tasks/2_blocks.yaml", "r") as file:
        task_config = yaml.safe_load(file)

    model = UR3e()
    timing_model = TimingModel(model.get_home_ik_solution())
    from dt.dt_modules.timing_model.TaskTrajectoryTimingEstimatorv2 import TaskTrajectoryTimingEstimator

    task_estimator = TaskTrajectoryEstimator(model)
    traj_timing_est = TaskTrajectoryTimingEstimator(model, timing_model)

    task_with_timings2 = traj_timing_est.get_task_trajectory_timings(task_config)

    trajq, trajqd, trajqdd, time, des = task_estimator.estimate_trajectory(
        task_with_timings2, save_to_file=False)

    print(trajq.shape, time.shape, des.shape)
    print(time)

    
