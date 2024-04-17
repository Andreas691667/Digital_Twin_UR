from sys import path
path.append("../..")

from ur3e.ur3e import UR3e
import numpy as np
from config.grid_config import GRID_CONFIG
from TimingModel import TimingModel

class TimingThresholdEstimator:
    def __init__(self, model) -> None:
        self.robot_model = model
        self.is_in_home_position = True
        self.last_ik_solutions = None
        self.missing_block = None

    def set_missing_block(self, missing_block: int) -> None:
        self.missing_block = missing_block

    def __compute_ik_solutions (self, task_config) -> np.ndarray:
        """Computes the inverse kinematics solutions
            returns 3D matrix:
                row: block number
                column: the four positions for a task
                depth: solutions for each position
        """
        number_of_blocks: int = task_config[GRID_CONFIG.NO_BLOCKS] 
        solutions: np.ndarray = np.zeros(shape=(number_of_blocks, 4, 6))
        
        # For every block (4 coordinates) calculate IK (for 4 coordinates)
        for bn in range(number_of_blocks):
            origin = task_config[bn][GRID_CONFIG.ORIGIN]
            target = task_config[bn][GRID_CONFIG.TARGET]
            origin_q_start, origin_q, target_q_start, target_q = self.robot_model.compute_joint_positions_origin_target(
                origin, target
            )
            
            # set solutions
            solutions[bn, 0, :] = origin_q_start
            solutions[bn, 1, :] = origin_q
            solutions[bn, 2, :] = target_q_start
            solutions[bn, 3, :] = target_q
        
        # set last solutions
        self.last_ik_solutions = solutions

        return solutions

    def __get_distances_of_leading_axis(self, joint_positions: np.ndarray) -> np.ndarray:
        """
        Get sliding windows differences
        If input is NxM, then output is (N-1)xM and the leading axis of N-1 x 1
        """
        # sliding window differences
        joint_positions_differences = np.abs(joint_positions[:-1, :] - joint_positions[1:, :])
        leading_axis = np.argmax(joint_positions_differences, axis=1)
        differences_leading_axis = joint_positions_differences[np.arange(len(joint_positions_differences)), leading_axis]
        return differences_leading_axis, leading_axis

    def __get_durations_of_leading_axis(self, distances_leading_axis: np.ndarray) -> np.ndarray:
        """
        Calculates the durations of the leading axis's
        Input: Distances of the leading axes between each position
        Output: Time it takes to traverse those distances
        Note: This may be calculated differently for shorter movement, i.e. if it does not reach max speed
        """
        JOINT_SPEED_MAX_DEG = 60 # deg/s
        JOINT_ACCELERATION_DEG = 80 # deg/s^2
        GRAP_TIME = 0.7 + 2 # s # TODO: add 1 as network delays add the end! not here but in base func
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
        all_durations_des = []
        for _, distance in enumerate(distances_leading_axis):
            # Check if movement reaches max speed
            # Does not reach max speed
            if ACCELERATION_DIST >= distance/2: # If the acceleration distance is greater than half of the distance, then it does not reach max speed
                # Calculate time to reach half-distance
                # d = 1/2 * a * t^2 => t = sqrt{2d/a}
                duration_half_distance = np.sqrt((2*(distance/2))/JOINT_ACCELERATION_RAD)
                all_durations.extend([duration_half_distance, duration_half_distance])
                all_durations_des.extend(["ACC", "DEC"])
            
            # Reaches max speed
            else: 
                duration_of_constant_speed = ((distance-ACCELERATION_DIST*2) / JOINT_SPEED_MAX_RAD)
                all_durations.extend([ACCELERATION_TIME, duration_of_constant_speed, ACCELERATION_TIME])
                all_durations_des.extend(["ACC", "CONSTANT", "DEC"])

        # Calculate combined time
        all_durations.append(GRAP_TIME)
        all_durations_des.append("GRAP")
        # combined_time_estimation_task = np.sum(all_durations)
        return all_durations, all_durations_des


    def __get_duration_between_positions (self, joint_positions: np.ndarray, block_number: int) -> np.ndarray:
        """
        Input is joins_positions of NxM, where N are the number of positions and M are DOF
        Outputs the combined duration between each joint position and the individual durations
        """
        # Get the distances of the leading axis
        distances_of_leading_axis, leading_axis = self.__get_distances_of_leading_axis(joint_positions)
        
        # Get the durations of movements
        all_durations, des = self.__get_durations_of_leading_axis(distances_of_leading_axis) 
        
        # Add delay
        if block_number != 0:
            all_durations.insert(-1, 0.3)
            des.append("HALF RELEASE")
            all_durations.append(0.5)
            des.append("FULL RELEASE")
            all_durations.append(0.5)
            des.append("DELAY")
            

        # Sum the durations
        combined_duration = np.sum(all_durations)

        return combined_duration, all_durations, des, leading_axis

    def __get_home_ik_sol(self):
        """Compute IK solution for home position"""
        HOME_X = GRID_CONFIG.HOME_POSITION[GRID_CONFIG.x]
        HOME_Y = GRID_CONFIG.HOME_POSITION[GRID_CONFIG.y]
        home_sol = self.robot_model.compute_joint_positions_xy(HOME_X, HOME_Y)
        return home_sol
    
    def __update_task(self, thresholds, task):
        """Update task with thresholds"""
        for i in range(len(thresholds)):
            task[i].update({GRID_CONFIG.TIMING_THRESHOLD: thresholds[i]})


    def compute_thresholds (self, task_config):
        """Computes the thresholds corresponding to block movements
        see __compute_ik_solutions for input format
        """
        ik_solutions = self.__compute_ik_solutions(task_config)
        number_of_blocks, _, _ = np.shape(ik_solutions)
        
        # Initialize threshold
        thresholds = []
        all_durations = []
        all_durations_des = []
        all_leading_axis = []
    
        
        # Calculate thresholds for blocks, if there is more than one block to move
        for block_number in range(number_of_blocks):
            # Get positions which matters for the timing calculations
            # That is: All subtasks jp of the first task, and the first subtask jp of the next task
            timing_essential_positions = []
            
            #missing block
            if block_number == self.missing_block:
                # Set threshold[block_number/missing_block] = <time from last origin to new origin>
                old_origin_grip_pos = self.last_ik_solutions[block_number, 1, :]
                current_origins = ik_solutions[block_number, :2, :]
                timing_essential_positions = np.vstack((old_origin_grip_pos, current_origins))
            
            # First block (or last if there are only one)
            elif block_number == 0 and self.is_in_home_position:
                home_sol = self.__get_home_ik_sol()
                origin_positions = ik_solutions[block_number, :2, :]
                timing_essential_positions = np.vstack((home_sol, origin_positions))
                
                # set home position to false
                self.is_in_home_position = False
            
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
            combined_duration, durations, des, leading_axis = self.__get_duration_between_positions(timing_essential_positions, block_number)
            thresholds.append(combined_duration)
            all_durations.extend(durations)
            all_durations_des.extend(des)
            all_leading_axis.extend(leading_axis)
        
        self.__update_task(thresholds, task_config)
        # print(f"Leading axis of movements: {all_leading_axis}")    
        return task_config, thresholds, all_durations, all_durations_des
    
class TimingThresholdEstimatorNew(TimingModel):
    def __init__(self, model) -> None:
        super().__init__(model)
        self.is_in_home_position = True

    def set_missing_block(self, missing_block: int) -> None:
        self.missing_block = missing_block

    
    def update_task(self, thresholds, task):
        """Update task with thresholds"""
        for i in range(len(thresholds)):
            task[i].update({GRID_CONFIG.TIMING_THRESHOLD: thresholds[i]})

    def get_timing_essential_positions (self, ik_solutions, block_number, number_of_blocks, missing_block):
        """Get timing essential joint positions"""
        # TODO: store ik_solutions in an attribute
        timing_essential_positions = []
        
        # missing block
        if block_number == missing_block:
            # Set threshold[block_number/missing_block] = <time from last origin to new origin>
            old_origin_grip_pos = self.last_ik_solutions[block_number, 1, :]
            current_origins = ik_solutions[block_number, :2, :]
            timing_essential_positions = np.vstack((old_origin_grip_pos, current_origins))
        
        # First block (or last if there are only one)
        elif block_number == 0 and self.is_in_home_position:
            home_sol = self.get_home_ik_sol()
            origin_positions = ik_solutions[block_number, :2, :]
            timing_essential_positions = np.vstack((home_sol, origin_positions))
            self.is_in_home_position = False
        
        # Middle block
        else:
            joint_positions_current_block = ik_solutions[block_number-1, :, :]
            joint_positions_next_block = ik_solutions[block_number, :, :]
            BGP = joint_positions_current_block[0, :]
            GP = joint_positions_current_block[1, :] 
            BTP = joint_positions_current_block[2, :] 
            TP = joint_positions_current_block[3, :] 
            origins_next = joint_positions_next_block[:2, :] # Origin before grip and grip pos
            timing_essential_positions = np.vstack((GP, BGP, BTP, TP, BTP, origins_next))
        
        return timing_essential_positions

    def compute_thresholds (self, task_config,  missing_block = -1):
        """Computes the thresholds corresponding to block movements
        see __compute_ik_solutions for input format
        """
        ik_solutions = self.compute_ik_solutions(task_config)
        number_of_blocks, _, _ = np.shape(ik_solutions)
        
        # Initialize threshold
        thresholds = []
        all_durations = []
        all_durations_des = []
        
        # Calculate thresholds for blocks, if there is more than one block to move
        for block_number in range(number_of_blocks):
            # Get positions which matters for the timing calculations
            # That is: All subtasks jp of the first task, and the first subtask jp of the next task
            timing_essential_positions = self.get_timing_essential_positions(ik_solutions, block_number, number_of_blocks, missing_block)
            
            # Get durations in between positions
            _, speed_profiles_ti_extended, speed_profiles_ti = self.get_duration_between_positions(timing_essential_positions, block_number)
            
             # Add additional ties
            _, speed_profiles_ti_with_delays = self.add_additional_ti(speed_profiles_ti_extended, speed_profiles_ti, block_number, number_of_blocks, -1)
            
            durations = self.strl2floatl(speed_profiles_ti_with_delays[0, :])
            des = speed_profiles_ti_with_delays[1, :]
            
            thresholds.append(np.sum(durations))
            all_durations.extend(durations)
            all_durations_des.extend(des)
        
        self.update_task(thresholds, task_config)
        # print(f"Leading axis of movements: {all_leading_axis}")    
        return task_config, thresholds, all_durations, all_durations_des

if __name__ == "__main__":
    import sys
    import yaml

    sys.path.append("../..")
    from ur3e.ur3e import UR3e

    with open(f"../../config/tasks/2_blocks.yaml", "r") as file:
        task_config = yaml.safe_load(file)

    model = UR3e()
    timing_threshold_estimator = TimingThresholdEstimator(model)
    new_task, thresholds, _, _ =  timing_threshold_estimator.compute_thresholds(task_config)
    timing_threshold_estimator_new = TimingThresholdEstimatorNew(model)
    new_task_new, thresholds_new, _, _ = timing_threshold_estimator_new.compute_thresholds(task_config)
    
    print(thresholds)
    print(thresholds_new)
    print(np.array_equal(thresholds, thresholds_new))
