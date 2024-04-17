from sys import path
path.append("../..")

import numpy as np
from config.grid_config import GRID_CONFIG
from dataclasses import dataclass

@dataclass 
class TIMING_INTERVALS:
    JOINT_SPEED_MAX_DEG = 60 # deg/s
    JOINT_ACCELERATION_DEG = 80 # deg/s^2
    GRAP_TIME = 0.7 # TODO: 2 seconds might need to be added
    GRAP_TIME_STR = str(GRAP_TIME) # TODO: 2 seconds might need to be added
    JOINT_SPEED_MAX_RAD = np.deg2rad(JOINT_SPEED_MAX_DEG) # rad/s
    JOINT_ACCELERATION_RAD = np.deg2rad(JOINT_ACCELERATION_DEG) # rad/s^2
    ACCELERATION_TIME = JOINT_SPEED_MAX_RAD / JOINT_ACCELERATION_RAD # Time it takes to reach maximun velocity
    ACCELERATION_DIST = 1/2 * JOINT_ACCELERATION_RAD * ACCELERATION_TIME**2
    PARTLY_OPEN_GRIPPER = 0.3
    PARTLY_OPEN_GRIPPER_STR = str(PARTLY_OPEN_GRIPPER)
    FULLY_OPEN_GRIPPER = 1.1
    FULLY_OPEN_GRIPPER_STR = str(FULLY_OPEN_GRIPPER)
    INITALIZATION_DELAY = 0.5
    INITALIZATION_DELAY_STR = str(INITALIZATION_DELAY)
    
    class TYPES:
        ACC = "ACCELERATION"
        DEC = "DEC"
        CON = "CON"
        PO = "PO"
        FO = "FO"
        DEL = "INITALIZATION_DELAY"
        TRI = "TRIP"
        TRAP = "TRAP" 
        DEL_ALL = "DEL_ALL"

class TimingModel:
    def __init__(self, model) -> None:
        self.robot_model = model
        self.last_ik_solutions = None
    
    def compute_ik_solutions (self, task_config) -> np.ndarray:
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

    def get_distances_of_leading_axis(self, joint_positions: np.ndarray) -> np.ndarray:
        """
        Get sliding windows differences
        If input is NxM, then output is (N-1)xM and the leading axis of N-1 x 1
        """
        # sliding window differences
        joint_positions_differences = np.abs(joint_positions[:-1, :] - joint_positions[1:, :])
        leading_axis = np.argmax(joint_positions_differences, axis=1)
        differences_leading_axis = joint_positions_differences[np.arange(len(joint_positions_differences)), leading_axis]
        return differences_leading_axis, leading_axis

    def get_durations_of_leading_axis(self, distances_leading_axis: np.ndarray) -> np.ndarray:
        """
        Calculates the durations of the leading axis's
        Input: Distances of the leading axes between each position
        Output: Time it takes to traverse those distances
        Note: This may be calculated differently for shorter movement, i.e. if it does not reach max speed
        """
        JOINT_SPEED_MAX_RAD = TIMING_INTERVALS.JOINT_SPEED_MAX_RAD
        JOINT_ACCELERATION_RAD = TIMING_INTERVALS.JOINT_ACCELERATION_RAD
        ACCELERATION_TIME = TIMING_INTERVALS.ACCELERATION_TIME
        ACCELERATION_DIST = TIMING_INTERVALS.ACCELERATION_DIST
        
        # For every distance the robot have to travel it does:
        # 1) Move with constant acceleration
        # 2) Move with with constant max speed
        # 3) Move with constant decceleration
        # If the movement is short in distance, it might skip step 2.

        ti_s = []
        ti_s_des = []
        spd_pf_and_dels = []
        spd_pf_and_dels_des = []

        for _, distance in enumerate(distances_leading_axis):
            # Check if movement reaches max speed
            # Does not reach max speed
            if ACCELERATION_DIST >= distance/2: # If the acceleration distance is greater than half of the distance, then it does not reach max speed
                # Calculate time to reach half-distance
                # d = 1/2 * a * t^2 => t = sqrt{2d/a}
                duration_half_distance = np.sqrt((2*(distance/2))/JOINT_ACCELERATION_RAD)
                
                # Add data
                ti_s.extend([duration_half_distance, duration_half_distance])
                ti_s_des.extend(["ACC", "DEC"])

                spd_pf_and_dels.extend([duration_half_distance + duration_half_distance])
                spd_pf_and_dels_des.extend(["MOVE"])
            
            # Reaches max speed
            else: 
                duration_of_constant_speed = ((distance-ACCELERATION_DIST*2) / JOINT_SPEED_MAX_RAD)
                
                # Add data
                ti_s.extend([ACCELERATION_TIME, duration_of_constant_speed, ACCELERATION_TIME])
                ti_s_des.extend(["ACC", "CON", "DEC"])

                spd_pf_and_dels.extend([ACCELERATION_TIME + duration_of_constant_speed + ACCELERATION_TIME])
                spd_pf_and_dels_des.extend(["MOVE"])

        
        # Format
        ti_s_formatted = np.vstack((ti_s, ti_s_des))
        spdpf_and_dels_formatted = np.vstack((spd_pf_and_dels, spd_pf_and_dels_des))
        
        return ti_s_formatted, spdpf_and_dels_formatted

    def get_duration_between_positions (self, joint_positions: np.ndarray, block_number: int) -> np.ndarray:
        """
        Input is joins_positions of NxM, where N are the number of positions and M are DOF
        Outputs the combined duration between each joint position and the individual durations
        """
        # Get the distances of the leading axis
        distances_of_leading_axis, leading_axis = self.get_distances_of_leading_axis(joint_positions)
        
        # Get timing intervals and speedprofiles/delays + descriptions
        ti_s, spdpf_and_dels = self.get_durations_of_leading_axis(distances_of_leading_axis)     

        # Sum the durations
        combined_duration = np.sum(self.strl2floatl(ti_s[0, :]))

        return combined_duration, ti_s, spdpf_and_dels

    def get_home_ik_sol(self):
        """Compute IK solution for home position"""
        HOME_X = GRID_CONFIG.HOME_POSITION[GRID_CONFIG.x]
        HOME_Y = GRID_CONFIG.HOME_POSITION[GRID_CONFIG.y]
        home_sol = self.robot_model.compute_joint_positions_xy(HOME_X, HOME_Y)
        return home_sol

    def add_additional_ti (self, speed_profiles_ti_extended, speed_profiles_ti, block_number, number_of_blocks,missing_block):
        """To be implemented"""
        speed_profiles_ti_extended = np.copy(speed_profiles_ti_extended)
        speed_profiles_ti = np.copy(speed_profiles_ti)
        _, number_of_speed_profiles_ti_extended = np.shape(speed_profiles_ti_extended)
        _, number_of_speed_profiles_ti = np.shape(speed_profiles_ti)
        

        # missing block
        if block_number == missing_block:
            pass
            
        # First block
        elif block_number == 0:
            indexes = [-2, number_of_speed_profiles_ti]
            elements = np.array([[str(TIMING_INTERVALS.FULLY_OPEN_GRIPPER-(0.4)-(0.1)), TIMING_INTERVALS.TYPES.DEL_ALL], 
                                 [TIMING_INTERVALS.GRAP_TIME_STR, TIMING_INTERVALS.TYPES.DEL_ALL]])
        
        # Middle block
        elif block_number != number_of_blocks:
            indexes = [-3, -2, number_of_speed_profiles_ti]
            elements = np.array([
                                [TIMING_INTERVALS.PARTLY_OPEN_GRIPPER_STR, TIMING_INTERVALS.TYPES.DEL_ALL],
                                [str(TIMING_INTERVALS.FULLY_OPEN_GRIPPER + TIMING_INTERVALS.INITALIZATION_DELAY), TIMING_INTERVALS.TYPES.DEL_ALL],
                                [TIMING_INTERVALS.GRAP_TIME_STR, TIMING_INTERVALS.TYPES.DEL_ALL]
                                ])
        # Last move
        else:
            indexes = [-2]
            elements = np.array([
                                [str(TIMING_INTERVALS.PARTLY_OPEN_GRIPPER+0.3), TIMING_INTERVALS.TYPES.DEL_ALL]
                                ])

        speed_profiles_ti = self.insert_at_indexes(speed_profiles_ti, elements, indexes)
           
        return -1, speed_profiles_ti
    
    def insert_at_indexes (self, array, elements, indexes):
        """Insert at indexes in ndarray, indexes must be negative and sorted from lowest to highest"""
        # Elements must be in the rows
        # Sort indexes and elements
        arr_out = array
        for i, _ in enumerate(indexes):
            arr_out = np.insert(arr_out, indexes[i], elements[i], axis=1)
            # Add to indexes
            if i != len(indexes):
                for j in range(i, len(indexes)):
                    if indexes[j] >= 0:
                        indexes[j] += 1
                # indexes[i+1:] = indexes[i+1:] + np.ones((len(indexes[i+1:])), dtype=int) 

        return arr_out

    def strl2floatl (self, str_list: np.ndarray) -> np.ndarray: 
        """Convert string list to float list
            TODO: Move this to a utils class
        """
        return [float(e) for e in str_list]





