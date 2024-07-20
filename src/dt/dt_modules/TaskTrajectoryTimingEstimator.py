from sys import path
path.append("../..")
import numpy as np

class TaskTrajectoryTimingEstimator():
    """TODO: Add description"""

    def __init__(self, robot_model, timing_model) -> None:
        self.robot_model = robot_model
        self.timing_model = timing_model

    def get_task_trajectory_timings (self, task_config, start_block=0, initializing=True):
        """Returns task trajectory timings in format:
            [FROM, TO, TI_VALUE, TI_TYPE]
        """
        ik_task_tensor = self.robot_model.compute_ik_task_tensor(task_config)
        self.timing_model.set_ik_solution_tensor(ik_task_tensor)
        self.timing_model.compute_timing_intervals(start_block, initializing)
        NUMBER_OF_TOTAL_TIs, _ = np.shape(self.timing_model.TI_matrix)
        TI_matrix_without_block_number = [self.timing_model.TI_matrix[i][1:] for i in range(NUMBER_OF_TOTAL_TIs)]
        return TI_matrix_without_block_number

# if __name__ == "__main__":
#     import yaml
#     path.append("../../..")
#     from models.robot_model.ur3e import UR3e # for testing!
#     from models.timing_model.TimingModel import TimingModel

#     with open(f"../../../config/tasks/2_blocks.yaml", "r") as file:
#         task_config = yaml.safe_load(file)

#     robot_model = UR3e()
#     timing_model = TimingModel(robot_model.get_home_ik_solution())
#     task_trajectory_timing_estimator = TaskTrajectoryTimingEstimator(robot_model, timing_model)
#     M = task_trajectory_timing_estimator.get_task_trajectory_timings(task_config)
#     # print(M)