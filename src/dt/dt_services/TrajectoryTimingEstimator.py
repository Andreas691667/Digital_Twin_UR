from sys import path
path.append("../..")

from ur3e.ur3e import UR3e
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
    
    # class DELAYS:
    #     class CASES:
    #         FROM_HOME = "FROM_HOME"
    #         DEFAULT = "DEFAULT"
    #         MISSING_BLOCK = "MISSING_BLOCK"
    #     class TYPES:
    #         ELEMENTS = "ELEMENTS"
    #         INDEXES = "INDEXES" 

class TrajectoryTimingEstimator:
    def __init__(self, model) -> None:
        self.robot_model = model
        self.is_in_home_position = True
        self.last_ik_solutions = None

        # self.DELAY_TYPES = TIMING_INTERVALS.DELAYS.TYPES
        # self.DELAY_CASES = TIMING_INTERVALS.DELAYS.CASES
        # self.delays = {
        #     self.DELAY_CASES.DEFAULT
        #         : {
        #             self.DELAY_TYPES.ELEMENTS: 
        #                 np.array([
        #                     [TIMING_INTERVALS.PARTLY_OPEN_GRIPPER_STR, TIMING_INTERVALS.TYPES.DEL_ALL],
        #                     [str(TIMING_INTERVALS.FULLY_OPEN_GRIPPER + TIMING_INTERVALS.INITALIZATION_DELAY), TIMING_INTERVALS.TYPES.DEL_ALL],
        #                     [TIMING_INTERVALS.GRAP_TIME_STR, TIMING_INTERVALS.TYPES.DEL_ALL]
        #                 ]),
        #             self.DELAY_TYPES.INDEXES: [-3, -2, number_of_speed_profiles_ti]
        #         },
        #     TIMING_INTERVALS.DELAYS.TYPES.FROM_HOME
        #         : np.array([
        #                 []
        #         ])
        # }
            
                
        


    def compute_ik_solutions (self, task_config):
        return self.__compute_ik_solutions(task_config)
    
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
        return self.__get_duration_between_positions(joint_positions, block_number)

    def __get_duration_between_positions (self, joint_positions: np.ndarray, block_number: int) -> np.ndarray:
        """
        Input is joins_positions of NxM, where N are the number of positions and M are DOF
        Outputs the combined duration between each joint position and the individual durations
        """
        # Get the distances of the leading axis
        distances_of_leading_axis, leading_axis = self.__get_distances_of_leading_axis(joint_positions)
        
        # Get timing intervals and speedprofiles/delays + descriptions
        ti_s, spdpf_and_dels = self.__get_durations_of_leading_axis(distances_of_leading_axis)     

        # Sum the durations
        combined_duration = np.sum(self.__strl2floatl(ti_s[0, :]))

        return combined_duration, ti_s, spdpf_and_dels

    def __get_home_ik_sol(self):
        """Compute IK solution for home position"""
        HOME_X = GRID_CONFIG.HOME_POSITION[GRID_CONFIG.x]
        HOME_Y = GRID_CONFIG.HOME_POSITION[GRID_CONFIG.y]
        home_sol = self.robot_model.compute_joint_positions_xy(HOME_X, HOME_Y)
        return home_sol

    def compute_thresholds (self, task_config,  missing_block = -1):
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
            if block_number == missing_block:
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
        
        
        print(f"Leading axis of movements: {all_leading_axis}")    
        return thresholds, all_durations, all_durations_des

    def __insert_at_indexes (self, array, elements, indexes):
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

    def __add_additional_ti (self, speed_profiles_ti_extended, speed_profiles_ti, block_number, number_of_blocks,missing_block):
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
            indexes = [number_of_speed_profiles_ti]
            elements = np.array([[TIMING_INTERVALS.GRAP_TIME_STR, TIMING_INTERVALS.TYPES.DEL_ALL]])
        
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
            print(speed_profiles_ti)
            indexes = [-2, -1]
            elements = np.array([
                                [TIMING_INTERVALS.PARTLY_OPEN_GRIPPER_STR, TIMING_INTERVALS.TYPES.DEL_ALL],
                                [TIMING_INTERVALS.FULLY_OPEN_GRIPPER_STR, TIMING_INTERVALS.TYPES.DEL_ALL],
                                ])

        speed_profiles_ti = self.__insert_at_indexes(speed_profiles_ti, elements, indexes)
           
        return -1, speed_profiles_ti

    def __get_timing_essential_positions (self, ik_solutions, block_number, number_of_blocks, missing_block):
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
        elif block_number == 0:
            home_sol = self.__get_home_ik_sol()
            origin_positions = ik_solutions[block_number, :2, :]
            timing_essential_positions = np.vstack((home_sol, origin_positions))
        
        # Middle block
        elif block_number != number_of_blocks:
            joint_positions_current_block = ik_solutions[block_number-1, :, :]
            joint_positions_next_block = ik_solutions[block_number, :, :]
            BGP = joint_positions_current_block[0, :]
            GP = joint_positions_current_block[1, :] 
            BTP = joint_positions_current_block[2, :] 
            TP = joint_positions_current_block[3, :] 
            origins_next = joint_positions_next_block[:2, :] # Origin before grip and grip pos
            timing_essential_positions = np.vstack((GP, BGP, BTP, TP, BTP, origins_next))

        # Last move
        else:
            joint_positions_current_block = ik_solutions[block_number-1, :, :]
            BGP = joint_positions_current_block[0, :]
            GP = joint_positions_current_block[1, :] 
            BTP = joint_positions_current_block[2, :] 
            TP = joint_positions_current_block[3, :] 
            HOME = self.__get_home_ik_sol()
            timing_essential_positions = np.vstack((GP, BGP, BTP, TP, BTP, HOME))
        
        return timing_essential_positions
    
    def __strl2floatl (self, str_list: np.ndarray) -> np.ndarray: 
        """Convert string list to float list
            TODO: Move this to a utils class
        """
        return [float(e) for e in str_list]

    def __format_task_with_timings (self, timing_essential_joint_positions: np.ndarray, speed_profiles_and_delays: np.ndarray) -> np.ndarray:
        """Formatting into task_with_timings format
        Params:
            timing_essential_joint_positions: (Nx6) where N is the number of essential joint positions
            speed_profiles_and_delays: 
            """
        # Initialize output matrix
        _, number_of_actions = np.shape(speed_profiles_and_delays)
        subtask_with_timings = np.empty(shape=(number_of_actions, 13))
        
        # get types and values
        types = speed_profiles_and_delays[1, :]
        values = self.__strl2floatl(speed_profiles_and_delays[0, :])

        # essential concatenated
        timing_essential_concat = np.hstack((timing_essential_joint_positions[:-1, :], timing_essential_joint_positions[1:, :]))
        
        # formatting
        j = 0 # move counter
        for i, type in enumerate (types):
            if type == "MOVE": #TODO add a type
                subtask_with_timings[i, :] = np.hstack((timing_essential_concat[j, :], values[i]))
                j += 1
            else:
                subtask_with_timings[i, :] = np.hstack(([16]*12, values[i]))
        
        return subtask_with_timings
    
    
    def get_traj_timings (self, task_config):
            """Computes the thresholds corresponding to block movements
            see __compute_ik_solutions for input format
            """
            ik_solutions = self.__compute_ik_solutions(task_config)
            number_of_blocks: int = task_config[GRID_CONFIG.NO_BLOCKS] 
            
            # Initialize threshold
            task_with_timings = np.array([])
        
            # Calculate thresholds for blocks, if there is more than one block to move
            for block_number in range(number_of_blocks+1):
                # Get positions which matters for the timing calculations
                # That is: All subtasks jp of the first task, and the first subtask jp of the next task
                timing_essential_positions = self.__get_timing_essential_positions(ik_solutions, block_number, number_of_blocks, -1)
                
                # Get speed profile timing intervals
                # TODO: Agree on this termonology througtout codebase!
                _, speed_profiles_ti_extended, speed_profiles_ti = self.__get_duration_between_positions(timing_essential_positions, block_number)
                
                # Add additional ties
                _, speed_profiles_ti_with_delays = self.__add_additional_ti(speed_profiles_ti_extended, speed_profiles_ti, block_number, number_of_blocks, -1)

                subtask_with_timings = self.__format_task_with_timings(timing_essential_positions, speed_profiles_ti_with_delays)

                # Add to task with timings
                if (len(task_with_timings) == 0):
                    task_with_timings = subtask_with_timings
                else:
                    task_with_timings = np.vstack((task_with_timings, subtask_with_timings)) 
                   
            return task_with_timings


if __name__ == "__main__":
    import sys
    import yaml

    sys.path.append("../..")
    from ur3e.ur3e import UR3e

    with open(f"../../config/tasks/2_blocks.yaml", "r") as file:
        task_config = yaml.safe_load(file)

    test = "get_traj_timings"
    model = UR3e()
    trajtimingestimator = TrajectoryTimingEstimator(model)
    ik_solutions = trajtimingestimator.compute_ik_solutions(task_config)
    
    if test == "format_task_with_timings":
        e = ik_solutions[0]
        timings = np.array([1, 2, 3])
        print(timings)
        print(e)
        new_format = trajtimingestimator.format_task_with_timings(e, timings)
        print(new_format)
        print(new_format.shape)
    
    elif test == "get_duration_between_positions":
        e = ik_solutions[0]
        combined_duration, ti_s, spdpf_and_dels = trajtimingestimator.get_duration_between_positions(e, -1)
        print(combined_duration)
        print(ti_s)
        print(spdpf_and_dels)
    
    elif test == "get_traj_timings":
        timingss = trajtimingestimator.get_traj_timings(task_config)
        print(timingss)



