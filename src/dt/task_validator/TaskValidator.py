import sys

sys.path.append("../..")
from config.task_config import TASK_CONFIG
import numpy as np
from time import sleep, time

class TaskValidator:
    """Class to validate a task a specified in TASK_CONFIG"""

    def __init__(self) -> None:
        self.block_origin_map = None
        self.block_target_map = None
        self.no_blocks = None
        self.task = None

        self.time_threshold = 2 # should handle new task in 2 second

    def validate_task(self, task: dict):
        """Validate the task
        params: task: <dict> - The task to validate
        returns: bool, dict - Whether the task is valid and the task
        If the task is valid, the returned task is the same as the input task"""

        # get attributes from task
        self.task = task
        self.__create_maps()

        # validate task
        valid = self.__validate_task()

        # parse matrices to task again
        self.__update_task()

        return valid, self.task

    def __update_task(self):
        """Update task from matrices"""
        for i in range(self.no_blocks):
            self.task[i][TASK_CONFIG.ORIGIN][TASK_CONFIG.x] = int(self.block_origin_map[0, i])
            self.task[i][TASK_CONFIG.ORIGIN][TASK_CONFIG.y] = int(self.block_origin_map[1, i])
            self.task[i][TASK_CONFIG.ORIGIN][TASK_CONFIG.ROTATE_WRIST] = bool(self.block_origin_map[2, i])
            self.task[i][TASK_CONFIG.TARGET][TASK_CONFIG.x] = int(self.block_target_map[0, i])
            self.task[i][TASK_CONFIG.TARGET][TASK_CONFIG.y] = int(self.block_target_map[1, i])
            self.task[i][TASK_CONFIG.TARGET][TASK_CONFIG.ROTATE_WRIST] = bool(self.block_target_map[2, i])

    def __create_maps(self):
        """Create matrices of block positions from task"""
        # get number of blocks
        self.no_blocks = self.task[TASK_CONFIG.NO_BLOCKS]

        # iterate over blocks to get maps of block positions as matrices
        self.block_origin_map = np.zeros((3, self.no_blocks))
        self.block_target_map = np.zeros((3, self.no_blocks))

        for i in range(self.no_blocks):
            self.block_origin_map[0, i] = self.task[i][TASK_CONFIG.ORIGIN][TASK_CONFIG.x]
            self.block_origin_map[1, i] = self.task[i][TASK_CONFIG.ORIGIN][TASK_CONFIG.y]
            self.block_origin_map[2, i] = self.task[i][TASK_CONFIG.ORIGIN][TASK_CONFIG.ROTATE_WRIST]
            self.block_target_map[0, i] = self.task[i][TASK_CONFIG.TARGET][TASK_CONFIG.x]
            self.block_target_map[1, i] = self.task[i][TASK_CONFIG.TARGET][TASK_CONFIG.y]
            self.block_target_map[2, i] = self.task[i][TASK_CONFIG.TARGET][TASK_CONFIG.ROTATE_WRIST]

    def __validate_task(self):
        """Validate the task by checking if the gripper can reach the target positions"""
        task_valid = False
        start_time = time()

        # continue until task is valid or time threshold is reached
        while (not task_valid) and (time() - start_time < self.time_threshold):
            # check if grip position is possible for targets
            task_valid = True
            # print(self.block_target_map)
            # print(self.block_origin_map)

            # check all blocks
            for i in range(self.no_blocks):

                # check if grip position is possible for targets
                grip_possible = self.__is_grip_possible(
                    self.block_target_map[:, i], self.block_target_map, 0, i
                )
                # print(f"Grip possible: {grip_possible}")

                # if not possible, change wrist rotation
                if not grip_possible:
                    task_valid = False
                    self.block_target_map[2, i] = 1 - self.block_target_map[2, i]

                # check if grip position is possible for origins
                grip_possible = self.__is_grip_possible(
                    self.block_origin_map[:, i], self.block_origin_map, i+1, self.no_blocks
                )
                # print(f"Grip possible: {grip_possible}")
                # if not possible, change wrist rotation
                if not grip_possible:
                    task_valid = False
                    self.block_origin_map[2, i] = 1 - self.block_origin_map[2, i]
            
        return task_valid

    def __is_grip_possible(self, block_pos, map, start, stop):
        """Check if a grip position is possible in given coordinate map"""

        grip_possible = True
        for block in range(start, stop):
            current_block_pos = map[:, block]
            if not np.equal(block_pos, current_block_pos).all(): # skip if same block
                # true distance between block and current block
                dist = (block_pos - current_block_pos) * TASK_CONFIG.GRID_PARAMETERS[TASK_CONFIG.HOLE_DIST]

                if block_pos[2] and dist[0] == 0:
                    if np.abs(dist[1]) <= TASK_CONFIG.GRIPPER_WIDTH:
                        grip_possible = False

                elif not block_pos[2] and dist[1] == 0:
                    if np.abs(dist[0]) <= TASK_CONFIG.GRIPPER_WIDTH:
                        grip_possible = False
            
        return grip_possible

# main
# if __name__ == "__main__":
#     task = TASK_CONFIG.block_config_close_blocks

#     print(task)

#     task_validator = TaskValidator()
#     valid, task_new = task_validator.validate_task(task)

#     print(f"Task valid: {valid}")
#     print(task_new)
