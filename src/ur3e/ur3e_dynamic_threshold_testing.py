from sys import path
from ur3e import UR3e
import numpy as np
path.append("..")

from config.task_config import TASK_CONFIG

class TimingThresholdEstimator:
    def __init__(self) -> None:
        self.robot_model = UR3e()

    def __compute_ik_solutions (self, task_config) -> np.ndarray:
        """Computes the inverse kinematics solutions
            returns 3D matrix:
                row: block number
                column: the four positions for a task
                depth: solutions for each position
        """
        number_of_blocks: int = task_config[TASK_CONFIG.NO_BLOCKS] 
        solutions: np.ndarray = np.zeros(shape=(number_of_blocks, 4, 6))
        
        # For every block (4 coordinates) calculate IK (for 4 coordinates)
        for bn in range(number_of_blocks):
            origin = task_config[bn][TASK_CONFIG.ORIGIN]
            target = task_config[bn][TASK_CONFIG.TARGET]
            origin_q_start, origin_q, target_q_start, target_q = self.robot_model.compute_joint_positions_origin_target(
                origin, target
            )
            
            # set solutions
            solutions[bn, 0, :] = origin_q_start
            solutions[bn, 1, :] = origin_q
            solutions[bn, 2, :] = target_q_start
            solutions[bn, 3, :] = target_q

        return solutions

    def __get_distances_of_leading_axis(self, joint_positions: np.ndarray) -> np.ndarray:
        """
        Get sliding windows differences
        If input is NxM, then output is (N-1)xM
        """
        # sliding window differences
        joint_positions_differences = np.abs(joint_positions[:-1, :] - joint_positions[1:, :])
        leading_axis = np.argmax(joint_positions_differences, axis=1)
        differences_leading_axis = joint_positions_differences[np.arange(len(joint_positions_differences)), leading_axis]
        return differences_leading_axis

    def __get_durations_of_leading_axis(self, distances_leading_axis: np.ndarray) -> np.ndarray:
        """
        Calculates the durations of the leading axis's
        Input: Distances of the leading axes between each position
        Output: Time it takes to traverse those distances
        Note: This may be calculated differently for shorter movement, i.e. if it does not reach max speed
        """
        JOINT_SPEED_MAX_DEG = 60 # deg/s
        JOINT_ACCELERATION_DEG = 80 # deg/s^2
        GRAP_TIME = 0.7 # s
        JOINT_SPEED_MAX_RAD = np.deg2rad(JOINT_SPEED_MAX_DEG) # rad/s
        JOINT_ACCELERATION_RAD = np.deg2rad(JOINT_ACCELERATION_DEG) # rad/s^2
        ACCELERATION_TIME = JOINT_SPEED_MAX_RAD / JOINT_ACCELERATION_RAD # Time it takes to reach maximun velocity
        ACCELERATION_DIST = 1/2 * JOINT_ACCELERATION_RAD * ACCELERATION_TIME**2
        
        # For every distance the robot have to travel it does:
        # 1) Move with constant acceleration
        # 2) Move with with constant max speed
        # 3) Move with constant decceleration
        # If the movement is short in distance, it might skip step 2.

        all_durations = []
        for _, distance in enumerate(distances_leading_axis):
            # Check if movement reaches max speed
            # Does not reach max speed
            if ACCELERATION_DIST >= distance/2: # If the acceleration distance is greater than half of the distance, then it does not reach max speed
                # Calculate time to reach half-distance
                # d = 1/2 * a * t^2 => t = sqrt{2d/a}
                duration_half_distance = np.sqrt((2*(distance/2))/JOINT_ACCELERATION_RAD)
                all_durations.extend([duration_half_distance, duration_half_distance])
            
            # Reaches max speed
            else: 
                duration_of_constant_speed = ((distance-ACCELERATION_DIST*2) / JOINT_SPEED_MAX_RAD)
                all_durations.extend([ACCELERATION_TIME, duration_of_constant_speed, ACCELERATION_TIME])

        # Calculate combined time
        all_durations.append(GRAP_TIME)
        combined_time_estimation_task = np.sum(all_durations)
        return combined_time_estimation_task, all_durations


    def __get_duration_between_positions (self, joint_positions: np.ndarray) -> np.ndarray:
        """
        Input is joins_positions of NxM, where N are the number of positions and M are DOF
        Outputs the combined duration between each joint position and the individual durations
        """
        # Get the distances of the leading axis
        distances_of_leading_axis = self.__get_distances_of_leading_axis(joint_positions)
        
        # Get the durations of movements
        durations_of_leading_axis, all_durations = self.__get_durations_of_leading_axis(distances_of_leading_axis) 
        
        # Sum the durations
        combined_duration = np.sum(durations_of_leading_axis)

        return combined_duration, all_durations

    def __get_home_ik_sol(self):
        """Compute IK solution for home position"""
        HOME_X = TASK_CONFIG.HOME_POSITION[TASK_CONFIG.x]
        HOME_Y = TASK_CONFIG.HOME_POSITION[TASK_CONFIG.y]
        home_sol = self.robot_model.compute_joint_positions_xy(HOME_X, HOME_Y)
        return home_sol

    def compute_thresholds (self, task_config):
        """Computes the thresholds corresponding to block movements
        see __compute_ik_solutions for input format
        """
        ik_solutions = self.__compute_ik_solutions(task_config)
        number_of_blocks, _, _ = np.shape(ik_solutions)
        
        # Initialize threshold
        thresholds = []
        all_durations = []
    
        
        # Calculate thresholds for blocks, if there is more than one block to move
        for block_number in range(number_of_blocks):
            # Get positions which matters for the timing calculations
            # That is: All subtasks jp of the first task, and the first subtask jp of the next task
            timing_essential_positions = []
            
            # First block (or last if there are only one)
            if block_number == 0:
                home_sol = self.__get_home_ik_sol()
                origin_positions = ik_solutions[block_number, :2, :]
                timing_essential_positions = np.vstack((home_sol, origin_positions))
            
            # Middle block or last block
            else:
                joint_positions_current_block = ik_solutions[block_number-1, :, :]
                joint_positions_next_block = ik_solutions[block_number, :, :]
                before_grip_joint_position = joint_positions_current_block[-2, :] # The gripper goes up before it moves on
                first_joint_positions_of_next_block = joint_positions_next_block[:2, :] # Origin before grip and grip pos
                timing_essential_positions = np.vstack((joint_positions_current_block, before_grip_joint_position, first_joint_positions_of_next_block))
            
            # Get durations in between positions
            # combined_duration: total time between timing_essential_positions
            # durations: durations between all moves
            combined_duration, durations = self.__get_duration_between_positions(timing_essential_positions)
            thresholds.append(combined_duration)
            all_durations.extend(durations)
            
            
        return thresholds, all_durations

if __name__ == "__main__":
    task_config = TASK_CONFIG.DYNAMIC_THRESHOLD_TWO_BLOCKS
    robot_model = UR3e()
    timing_threshold_estimator = TimingThresholdEstimator()
    thresholds, all_durations = timing_threshold_estimator.compute_thresholds(task_config)
    
    print(thresholds)
    print(all_durations)

    
