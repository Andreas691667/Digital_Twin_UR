
import numpy as np
import pandas as pd

class TaskTrajectoryEstimator:
    """Class to estimate the trajectory of a task."""

    def __init__(self, model) -> None:
        self.robot_model = model

    def estimate_trajectory(self, task_with_timings, save_to_file=False, file_name=None):
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
            start = None if (elem[0:6] == [None]*6).all() else elem[0:6]
            target = None if (elem[6:12] == [None]*6).all() else elem[6:12]
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
                time_vector = np.linspace(final_time[-1]+0.05, motion_time+final_time[-1], n_steps)
                final_time = np.append(final_time, time_vector, axis=0)
                last_traj = traj

            # If there is a delay
            elif start is None and target is None:
                final_trajectory = np.append(final_trajectory, np.tile(last_traj[-1], n_steps), axis=0)
                time_vector = np.linspace(final_time[-1]+0.05, motion_time+final_time[-1], n_steps)
                final_time = np.append(final_time, time_vector, axis=0)

        # Reshape the final trajectory
        final_trajectory = np.reshape(final_trajectory, (-1, 6))

        # Save the trajectory to a file
        if save_to_file:  
            df = pd.DataFrame(final_trajectory, columns=['actual_q_0','actual_q_1','actual_q_2','actual_q_3','actual_q_4','actual_q_5'])
            # add time column
            df['timestamp'] = final_time
            df.to_csv("trajectory_dt.csv", sep=' ', index=False)

        return final_trajectory, final_time
    

# Example of usage
# main
if __name__ == "__main__":
    import sys
    sys.path.append("../..")
    from ur3e.ur3e import UR3e
    model = UR3e()
    task_estimator = TaskTrajectoryEstimator(model)

    v_none= [None]*6
    task_with_timings = []

    # generate 4 random starts and targets and 4 associated timings
    # and create a 4x13 array
    for i in range(4):
        s = np.random.uniform(-np.pi, np.pi, 6)
        t = np.random.uniform(-np.pi, np.pi, 6)
        timing = np.random.uniform(0.5, 2)
        vec = np.concatenate((s, t, [timing]))
        task_with_timings.append(vec)

    vec = np.concatenate((v_none, v_none, [0.7]))
    task_with_timings.append(vec)
    

    print((np.shape(task_with_timings)))

    # print(task_with_timings)
    traj, time = task_estimator.estimate_trajectory(task_with_timings)

    print((traj))