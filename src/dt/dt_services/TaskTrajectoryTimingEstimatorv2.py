from TimingModelv2 import TimingModel
from sys import path
path.append("../..")
from ur3e.ur3e import UR3e # for testing!
import numpy as np

class TaskTrajectoryTimingEstimator(TimingModel):
    def __init__(self, robot_model) -> None:
        super().__init__(robot_model)

    def get_task_trajectory_timings (self, task_config, start_block=0):
        """Returns task trajectory timings in format:
            [FROM, TO, TI_VALUE, TI_TYPE]
        """
        ik_task_tensor = self.robot_model.compute_ik_task_tensor(task_config)
        self.set_ik_solution_tensor(ik_task_tensor)
        self.compute_timing_intervals(start_block)
        NUMBER_OF_TOTAL_TIs, _ = np.shape(self.TI_matrix)
        TI_matrix_without_block_number = [self.TI_matrix[i][1:] for i in range(NUMBER_OF_TOTAL_TIs)]
        return TI_matrix_without_block_number

if __name__ == "__main__":
    import yaml

    with open(f"../../config/tasks/2_blocks.yaml", "r") as file:
        task_config = yaml.safe_load(file)

    robot_model = UR3e()
    task_trajectory_timing_estimator = TaskTrajectoryTimingEstimator(robot_model)
    M = task_trajectory_timing_estimator.get_task_trajectory_timings(task_config)
    print(M)
