from sys import path
from ur3e import UR3e
import numpy as np
path.append("..")

from config.task_config import TASK_CONFIG


def compute_ik_solutions (task_config, robot_model) -> np.ndarray:
    """Computes the inverse kinematics solutions
        returns 3D matrix:
            row: block number
            column: the four positions for a task
            depth: solutions for each position
    """
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

def get_distances_of_leading_axis(joint_positions: np.ndarray) -> np.ndarray:
    # sliding window differences
    joint_positions_differences = np.abs(joint_positions[:-1, :] - joint_positions[1:, :])
    leading_axis = np.argmax(joint_positions_differences, axis=1)
    differences_leading_axis = joint_positions_differences[np.arange(len(joint_positions_differences)), leading_axis]
    return differences_leading_axis

def get_durations_of_leading_axis(differences_leading_axis: np.ndarray) -> np.ndarray:
    """Calculates the durations of the leading axis's
        Note: This may be calculated differently for shorter movement, i.e. if it does not reach max speed
    """
    JOINT_SPEED_MAX = 60 # deg/s
    JOINT_ACCELERATION = 80 # deg/s^2
    ACCELERATION_TIME = JOINT_SPEED_MAX / JOINT_ACCELERATION
    ACCELERATION_DIST = 1/2 * JOINT_ACCELERATION * ACCELERATION_TIME**2
    
    # duration of acceleration and deceleration of trapezoid
    duration_of_acc = 2 * ACCELERATION_TIME
    # duration of constant speed of trapezoid
    duration_of_constant_speed = ((differences_leading_axis-ACCELERATION_DIST) / JOINT_SPEED_MAX)
    # combined duration
    combined_time_estimation_task = duration_of_constant_speed + duration_of_acc
    return combined_time_estimation_task


def get_duration_between_positions (joint_positions: np.ndarray) -> np.ndarray:
    # Get the distances of the leading axis
    distances_of_leading_axis = get_distances_of_leading_axis(joint_positions)

    # Get the durations of movements
    durations_of_leading_axis = get_durations_of_leading_axis(distances_of_leading_axis) 

    # Sum the durations
    combined_duration = np.sum(durations_of_leading_axis)

    return combined_duration

def get_home_and_origin(ik_solutions, robot_model):
    HOME_X = TASK_CONFIG.HOME_POSITION[TASK_CONFIG.x]
    HOME_Y = TASK_CONFIG.HOME_POSITION[TASK_CONFIG.y]
    home_sol = robot_model.compute_joint_positions_xy(HOME_X, HOME_Y)
    origin_task0 = ik_solutions[0, 0, :]
    positions = np.vstack((home_sol, origin_task0))
    return positions

def compute_thresholds (ik_solutions: np.ndarray, robot_model):
    """Computes the thresholds corresponding to block movements
    see __compute_ik_solutions for input format
    """
    number_of_blocks, _, _ = np.shape(ik_solutions)

    # Initialize threshold with time it takes to go from home to next block
    home_and_origin = get_home_and_origin(ik_solutions, robot_model)
    thresholds = [get_duration_between_positions(home_and_origin)]
    print(f"Home -> Origin: {thresholds}")
    
    for block_number in range(number_of_blocks-1):
        joint_positions_current_block = ik_solutions[block_number, :, :]
        joint_positions_next_block = ik_solutions[block_number+1, :, :]
        
        # Get positions which matters for the timing calculations
        # That is: All subtasks jp of the first task, and the first subtask jp of the next task
        before_grip_joint_position = joint_positions_current_block[-2, :]
        first_joint_position_of_next_block = joint_positions_next_block[0, :]
        timing_essential_positions = np.vstack((joint_positions_current_block, before_grip_joint_position, first_joint_position_of_next_block))

        # Get durations in between positions
        duration = get_duration_between_positions(timing_essential_positions)
        thresholds.append(duration)
        
        
    return thresholds

if __name__ == "__main__":
    task_config = TASK_CONFIG.DYNAMIC_THRESHOLD_SMALL_TASK
    robot_model = UR3e()

    sol = compute_ik_solutions(task_config, robot_model)
    
    # Home, Origins, Targets
    positions = np.vstack((get_home_and_origin(sol, robot_model), sol[0, 1:, :]))
    
    timing = get_duration_between_positions(positions)
    print(timing)
    
    # thresholds = compute_thresholds(sol, robot_model)
    
    # # print(sol)
    # print(thresholds)
    # print(sol)
    # block_number = 0

    # origin = task_config[block_number][TASK_CONFIG.ORIGIN]
    # target = task_config[block_number][TASK_CONFIG.TARGET]

    # # Calculate joint positions
    # origin_q_start, origin_q, target_q_start, target_q = robot_model.compute_joint_positions_origin_target(
    #     origin, target
    # )

    # print(type(origin_q_start))

    
