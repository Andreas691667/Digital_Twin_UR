import sys
sys.path.append("../..")
from config.grid_config import GRID_CONFIG
import numpy as np
from time import time

class TaskValidator:
    """Class to validate a task a specified in GRID_CONFIG"""

    def __init__(self) -> None:
        self.block_origin_map = None
        self.block_target_map = None
        self.no_blocks = None
        self.task = None
        self.validating_threshold = 2 # should handle new task in 2 seconds
    
    def validate_task(self, task: dict, start_block: int = 0):
        """Validate the task
        params: task: <dict> - The task to validate
        returns: bool, dict - Whether the task is valid and the task
        If the task is valid, the returned task is the same as the input task"""

        # get attributes from task
        self.task = task
        self.__create_maps()

        # validate task
        valid = self.__validate_task(start_block)

        if valid: 
            # parse matrices to task again
            self.__update_task()
        return valid, self.task

    def __update_task(self):
        """Update task from matrices"""
        for i in range(self.no_blocks):
            self.task[i][GRID_CONFIG.ORIGIN][GRID_CONFIG.x] = int(self.block_origin_map[0, i])
            self.task[i][GRID_CONFIG.ORIGIN][GRID_CONFIG.y] = int(self.block_origin_map[1, i])
            self.task[i][GRID_CONFIG.ORIGIN].update({GRID_CONFIG.ROTATE_WRIST: bool(self.block_origin_map[2, i])})
            self.task[i][GRID_CONFIG.TARGET][GRID_CONFIG.x] = int(self.block_target_map[0, i])
            self.task[i][GRID_CONFIG.TARGET][GRID_CONFIG.y] = int(self.block_target_map[1, i])
            self.task[i][GRID_CONFIG.TARGET].update({GRID_CONFIG.ROTATE_WRIST: bool(self.block_target_map[2, i])})

    def __create_maps(self):
        """Create matrices of block positions from task"""
        # get number of blocks
        self.no_blocks = self.task[GRID_CONFIG.NO_BLOCKS]

        # iterate over blocks to get maps of block positions as matrices
        self.block_origin_map = np.zeros((3, self.no_blocks))
        self.block_target_map = np.zeros((3, self.no_blocks))

        for i in range(self.no_blocks):
            self.block_origin_map[0, i] = self.task[i][GRID_CONFIG.ORIGIN][GRID_CONFIG.x]
            self.block_origin_map[1, i] = self.task[i][GRID_CONFIG.ORIGIN][GRID_CONFIG.y]
            self.block_origin_map[2, i] = False
            self.block_target_map[0, i] = self.task[i][GRID_CONFIG.TARGET][GRID_CONFIG.x]
            self.block_target_map[1, i] = self.task[i][GRID_CONFIG.TARGET][GRID_CONFIG.y]
            self.block_target_map[2, i] = False

    def __validate_task(self, start_block):
        """Validate the task by checking if the gripper can reach the target positions"""
        task_valid = False
        start_time = time()

        # continue until task is valid or time threshold is reached
        while (not task_valid) and (time() - start_time < self.validating_threshold):
            # check if grip position is possible for targets
            task_valid = True

            # check all blocks
            for i in range(start_block, self.no_blocks):

                # check if grip position is possible for targets
                grip_possible = self.__is_grip_possible(
                    self.block_target_map[:, i], self.block_target_map, 0, i, GRID_CONFIG.GRIPPER_WIDTH_TARGET
                )

                # if not possible, change wrist rotation
                if not grip_possible:
                    task_valid = False
                    self.block_target_map[2, i] = 1 - self.block_target_map[2, i]
                    break

                # check if grip position is possible for origins
                grip_possible = self.__is_grip_possible(
                    self.block_origin_map[:, i], self.block_origin_map, i+1, self.no_blocks, GRID_CONFIG.GRIPPER_WIDTH_ORIGIN
                )

                # if not possible, change wrist rotation
                if not grip_possible:
                    task_valid = False
                    self.block_origin_map[2, i] = 1 - self.block_origin_map[2, i]
                    break

        return task_valid

    def __is_grip_possible(self, block_pos, block_map, start, stop, gripper_threshold):
        """Check if a grip position is possible in given coordinate map"""

        grip_possible = True
        for block in range(start, stop):
            current_block_pos = block_map[:, block]
            if not np.equal(block_pos, current_block_pos).all(): # skip if same block
                # true distance between block and current block
                dist = (block_pos - current_block_pos) * GRID_CONFIG.GRID_PARAMETERS[GRID_CONFIG.HOLE_DIST]

                if block_pos[2] and dist[0] == 0:
                    D = np.abs(dist[1]) - GRID_CONFIG.BLOCK_WIDTH/2
                    if D <= gripper_threshold/2:
                        grip_possible = False

                elif not block_pos[2] and dist[1] == 0:
                    D = np.abs(dist[0]) - GRID_CONFIG.BLOCK_WIDTH/2
                    if D <= gripper_threshold/2:
                        grip_possible = False

                # if block_pos[2] and dist[0] == 0:
                #     if np.abs(dist[1]) <= threshold:
                #         grip_possible = False

                # elif not block_pos[2] and dist[1] == 0:
                #     if np.abs(dist[0]) <= threshold:
                #         grip_possible = False

        return grip_possible

# main
if __name__ == "__main__":
    import yaml
    import json
    # task = GRID_CONFIG.block_config_close_blocks

    with open("../../config/tasks/2_blocks.yaml", "r", encoding="utf-8") as file:
        task = yaml.safe_load(file)


    task_validator = TaskValidator()
    valid, task_new = task_validator.validate_task(task)

    print(f"Task valid: {valid}")
    print(json.dumps(task_new, indent=4))
