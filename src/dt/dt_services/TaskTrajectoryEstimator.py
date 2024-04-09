import numpy as np
import pandas as pd
import datetime


class TaskTrajectoryEstimator:
    """Class to estimate the trajectory of a task."""

    def __init__(self, model) -> None:
        self.robot_model = model

    def estimate_trajectory(
        self, task_with_timings, save_to_file=False, file_name=None
    ):
        """Estimate the trajectory of a task.
        params:
            task_with_timings: Array with all starts and ends of the task and the timing intervals.
            type task_with_timings: Mx13 array where M=N+D, N is the number of start-target-waypoints and D is the number delays
        returns:
            Estimated trajectory."""

        final_trajectory = []
        final_time = [0]
        last_traj = []

        for elem in task_with_timings:
            # Get the start, target and motion time
            start = None if (elem[0:6] == [None] * 6).all() else elem[0:6]
            target = None if (elem[6:12] == [None] * 6).all() else elem[6:12]
            motion_time = elem[12]

            # Calculate the trajectory
            self.robot_model.set_motion_time(motion_time)
            n_steps = self.robot_model.set_n_steps_motion()

            # check that start and target is not None
            if start is not None and target is not None:
                # Compute the trajectory
                traj = self.robot_model.compute_trajectory(start, target)
                traj_q = traj.q

                final_trajectory = np.append(final_trajectory, traj_q)

                # Compute the time
                time_vector = np.linspace(
                    final_time[-1] + 0.05, motion_time + final_time[-1], n_steps
                )
                final_time = np.append(final_time, time_vector, axis=0)
                last_traj = traj

            # If there is a delay
            elif start is None and target is None:
                final_trajectory = np.append(
                    final_trajectory, np.tile(last_traj.q[-1], n_steps), axis=0
                )
                time_vector = np.linspace(
                    final_time[-1] + 0.05, motion_time + final_time[-1], n_steps
                )
                final_time = np.append(final_time, time_vector, axis=0)

        # Reshape the final trajectory
        final_trajectory = np.reshape(final_trajectory, (-1, 6))

        # Save the trajectory to a file
        if save_to_file:
            df = pd.DataFrame(
                final_trajectory,
                columns=[
                    "actual_q_0",
                    "actual_q_1",
                    "actual_q_2",
                    "actual_q_3",
                    "actual_q_4",
                    "actual_q_5",
                ],
            )
            # add time column
            final_time = np.delete(final_time, 0)
            df["timestamp"] = final_time
            file_name = (
                file_name
                if file_name is not None
                else datetime.datetime.now().strftime("%m_%d_%Y_%H_%M_%S")
            )
            df.to_csv(f"../dt_trajectories/{file_name}.csv", sep=" ", index=False)

        return final_trajectory, final_time


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

    origin0 = task_config[0]['ORIGIN']
    target0 = task_config[0]['TARGET']
    origin1 = task_config[1]['ORIGIN']
    target1 = task_config[1]['TARGET']
    HOME = model.compute_joint_positions_xy(11, -2)
    BGP0, GP0, BTP0, TP0 = model.compute_joint_positions_origin_target(origin0, target0)
    BGP1, GP1, BTP1, TP1 = model.compute_joint_positions_origin_target(origin1, target1)
    v_none = [None] * 6 

    # Create a task with timings
    task_with_timings = [
        # np.concatenate((v_none,v_none, [3])),
        np.concatenate((HOME, BGP0, [1.5])),
        np.concatenate((BGP0, GP0, [1.5])),
        np.concatenate((v_none,v_none, [0.7])),
        np.concatenate((GP0, BGP0, [1.5])),
        np.concatenate((BGP0, BTP0, [1.5])),
        np.concatenate((BTP0, TP0, [1.5])),
        np.concatenate((v_none,v_none, [0.3])),
        np.concatenate((TP0, BTP0, [1.5])),
        np.concatenate((v_none,v_none, [1.1])),
        np.concatenate((BTP0, BGP1, [1.5])),
        np.concatenate((BGP1, GP1, [1.5])),
        np.concatenate((v_none,v_none, [1.1])),
        np.concatenate((GP1, BGP1, [1.5])),
        np.concatenate((BGP1, BTP1, [1.5])),
        np.concatenate((BTP1, TP1, [1.5])),
        np.concatenate((v_none,v_none, [0.3])),
        np.concatenate((TP1, BTP1, [1.5])),
        np.concatenate((v_none,v_none, [1.1])),
        np.concatenate((TP1, HOME, [1.5])),
    ]

    traj, time = task_estimator.estimate_trajectory(task_with_timings, True)

    print((traj))
