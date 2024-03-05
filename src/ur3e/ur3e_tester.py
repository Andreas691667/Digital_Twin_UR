from sys import path
from ur3e import UR3e
import numpy as np
path.append("..")

from config.task_config import TASK_CONFIG


def compute_ik_solutions (task_config, robot_model) -> np.ndarray:
    """Computes the inverse kinematics solutions"""
    number_of_blocks: int = task_config[TASK_CONFIG.NO_BLOCKS] 
    solutions: np.ndarray = np.zeros(shape=(number_of_blocks, 4, 6))
    print(solutions.shape)
    # For every block (4 coordinates) calculate IK (for 4 coordinates)
    print(number_of_blocks)
    
    for bn in range(number_of_blocks):
        origin = task_config[bn][TASK_CONFIG.ORIGIN]
        target = task_config[bn][TASK_CONFIG.TARGET]
        origin_q_start, origin_q, target_q_start, target_q = robot_model.compute_joint_positions_origin_target(
            origin, target
        )
        
        # set solutions
        solutions[bn, 0, :] = origin_q_start
        solutions[bn, 1, :] = origin_q
        solutions[bn, 2, :] = target_q_start
        solutions[bn, 3, :] = target_q

    return solutions

def compute_thresholds (ik_solutions: np.ndarray):
    """Computes the thresholds corresponding to block movements
    see __compute_ik_solutions for input format
    """
    # Based on largest distance between joint positions
    JOINT_SPEED_MAX = 60 # rad/s
    JOINT_ACCELERATION = 80 # rad/s^2
    RISE_TIME = JOINT_SPEED_MAX / JOINT_ACCELERATION

    number_of_blocks, _, _ = np.shape(ik_solutions)
    
    for block_number in range(number_of_blocks-1):
        joint_positions_current_block = ik_solutions[block_number, :, :]
        joint_positions_next_block = ik_solutions[block_number+1, :, :]
        joint_positions_difference = np.abs(joint_positions_current_block-joint_positions_next_block)
        leading_axis = np.argmax(joint_positions_difference, axis=1)

        print(joint_positions_difference)
        print(leading_axis)

        # use leading axis of movement to calculate timing for each subtask in block
        combined_time_estimation_for_block = 0 # initialize to 0
        for subtask in range(4):
            subtask_leading_axis = leading_axis[subtask]
            radians_of_leading_axis = joint_positions_difference[subtask, subtask_leading_axis]
            print(f"radians subtask {subtask} of block {block_number} \n", radians_of_leading_axis)
            
            # calculate distance covered in rise time
            subtask_time = -1
            # calculate time for movement based on speed and acceleration

        # add combined to list
        
    return []

if __name__ == "__main__":
    task_config = TASK_CONFIG.block_config_heart
    robot_model = UR3e()

    print(TASK_CONFIG.RISE_TIME)
    print(TASK_CONFIG.RISE_DIST)
    # sol = compute_ik_solutions(task_config, robot_model)
    # thresholds = compute_thresholds(sol)

    # print(sol)
    # block_number = 0

    # origin = task_config[block_number][TASK_CONFIG.ORIGIN]
    # target = task_config[block_number][TASK_CONFIG.TARGET]

    # # Calculate joint positions
    # origin_q_start, origin_q, target_q_start, target_q = robot_model.compute_joint_positions_origin_target(
    #     origin, target
    # )

    # print(type(origin_q_start))

    
