# Imports
import numpy as np
from sys import path
from dataclasses import dataclass
path.append("../..")
from ur3e.ur3e import UR3e
from config.timing_config import ROBOT_PHYSICS
from config.timing_config import TI_TYPES
from config.timing_config import TI_CONSTANT_VALUES
from config.timing_config import ACTION_PROCEDURES

# TimingModel
class TimingModel:
    def __init__(self) -> None:
        # TI_matrix = [SUBTASK, FROM, TO, TI_VALUE, TI_TYPE], TI: "Timing Interval"
        # SUBTASK: The move of a block from its origin to a target
        # FROM: The joint position to move from
        # TO: The joint position to move to
        # TI_VALUE: "Timing Interval Value"
        # TI_TYPE: "Timing Interval Type"
        self.TI_matrix = []

        # Model
        self.robot_model = UR3e()
        self.missing_block = -1

    
    # ---- GETTER METHODS ----
    def get_timing_intervals (self) -> np.ndarray:
        return self.timing_intervals
    
    # ---- INTERNAL METHODS ----
    def _get_distances_of_leading_axis(self, joint_positions: np.ndarray) -> np.ndarray:
        """
        Get sliding windows differences
        If input is NxM, then output is (N-1)xM and the leading axis of N-1 x 1
        """
        # sliding window differences
        joint_positions_differences = np.abs(joint_positions[:-1, :] - joint_positions[1:, :])
        leading_axis = np.argmax(joint_positions_differences, axis=1)
        differences_leading_axis = joint_positions_differences[np.arange(len(joint_positions_differences)), leading_axis]
        return differences_leading_axis

    def _get_durations_of_leading_axis(self, distances_leading_axis: np.ndarray) -> np.ndarray:
        """
        Calculates the durations of the leading axis's
        Input: Distances of the leading axes between each position
        Output: Time it takes to traverse those distances
        Note: This may be calculated differently for shorter movement, i.e. if it does not reach max speed
        """
        # Initialize output
        durations = []

        # Get contants
        ACCELERATION_DIST = ROBOT_PHYSICS.ACCELERATION_DIST
        JOINT_ACCELERATION_RAD = ROBOT_PHYSICS.JOINT_ACCELERATION_RAD
        JOINT_SPEED_MAX_RAD = ROBOT_PHYSICS.JOINT_ACCELERATION_RAD
        ACCELERATION_TRAP_TIME = ROBOT_PHYSICS.ACCELERATION_TRAP_TIME
        
        # For every distance the robot have to travel it does:
        # 1) Move with constant acceleration
        # 2) Move with with constant max speed
        # 3) Move with constant decceleration
        # If the movement is short in distance, it might skip step 2.
        for _, distance in enumerate(distances_leading_axis):
            # Check if movement reaches max speed
            # Does not reach max speed
            if ACCELERATION_DIST >= distance/2: # If the acceleration distance is greater than half of the distance, then it does not reach max speed
                # Calculate time to reach half-distance
                # d = 1/2 * a * t^2 => t = sqrt{2d/a}
                TRI_ACC_TI = np.sqrt((2*(distance/2))/JOINT_ACCELERATION_RAD)
                TRI_TI = 2*TRI_ACC_TI
        
                # Add data
                durations.extend([TRI_TI])
            
            # Reaches max speed
            else: 
                TRAP_CON_TI = ((distance-ACCELERATION_DIST*2) / JOINT_SPEED_MAX_RAD)
                TRAP_TI = 2*ACCELERATION_TRAP_TIME + TRAP_CON_TI
                # Add data
                durations.extend([TRAP_TI])

        # Return  
        return durations


    def _get_duration_between_positions (self, joint_positions: np.ndarray) -> np.ndarray:
        """
        Input is joins_positions of NxM, where N are the number of positions and M are DOF
        Outputs the combined duration between each joint position and the individual durations
        """
        # Get the distances of the leading axis
        distances_of_leading_axis = self._get_distances_of_leading_axis(joint_positions)
        
        # Get timing intervals and speedprofiles/delays + descriptions
        durations = self._get_durations_of_leading_axis(distances_of_leading_axis)     

        return durations


    def _get_timing_essential_positions (self, action_procedure, block_number):
        """Get timing essential joint positions"""
        # TODO: store ik_solution_tensor in an attribute
        timing_essential_positions = []
        
        # missing block
        if action_procedure == ACTION_PROCEDURES.MISSING_BLOCK_TO_BLOCK_TO_BLOCK:
            # Set threshold[block_number/missing_block] = <time from last origin to new origin>
            old_origin_grip_pos = self.last_ik_solutions[block_number, 1, :]
            current_origins = self.ik_solution_tensor[block_number, :2, :]
            #TODO: TO BE FINISHED
            timing_essential_positions = np.vstack(())
        
        # First block (or last if there are only one)
        elif action_procedure == ACTION_PROCEDURES.HOME_TO_BLOCK_TO_BLOCK:
            joint_positions_current_block = self.ik_solution_tensor[block_number, :, :]
            joint_positions_next_block = self.ik_solution_tensor[block_number+1, :, :]
            HOME = self.robot_model.get_home_ik_solution()
            BGP = joint_positions_current_block[0, :]
            GP = joint_positions_current_block[1, :] 
            BTP = joint_positions_current_block[2, :] 
            TP = joint_positions_current_block[3, :]
            BGP_next = joint_positions_next_block[0, :]
            GP_next = joint_positions_next_block[1, :] 
            
            timing_essential_positions = np.vstack((HOME, BGP, GP, BGP, BTP, TP, BTP, BGP_next, GP_next))
        
        # Middle block
        elif action_procedure == ACTION_PROCEDURES.BLOCK_TO_BLOCK:
            joint_positions_current_block = self.ik_solution_tensor[block_number, :, :]
            joint_positions_next_block = self.ik_solution_tensor[block_number+1, :, :]
            BGP = joint_positions_current_block[0, :]
            GP = joint_positions_current_block[1, :] 
            BTP = joint_positions_current_block[2, :] 
            TP = joint_positions_current_block[3, :]
            BGP_next = joint_positions_next_block[0, :]
            GP_next = joint_positions_next_block[1, :] 
            
            timing_essential_positions = np.vstack((GP, BGP, BTP, TP, BTP, BGP_next, GP_next))

        # Last move
        else:
            joint_positions_current_block = self.ik_solution_tensor[block_number, :, :]
            BGP = joint_positions_current_block[0, :]
            GP = joint_positions_current_block[1, :] 
            BTP = joint_positions_current_block[2, :] 
            TP = joint_positions_current_block[3, :]
            HOME = self.robot_model.get_home_ik_solution()

            
            timing_essential_positions = np.vstack((GP, BGP, BTP, TP, BTP, HOME))
        
        return timing_essential_positions

    # ---- MAIN METHODS ----
    def set_ik_solution_tensor (self, ik_solution_tensor):
        """Sets ik solutions tensor"""
        self.ik_solution_tensor = ik_solution_tensor
    
    def set_missing_block (self, missing_block):
        """Sets missing_block"""
        self.missing_block = missing_block
    
    def compute_timing_intervals (self) -> np.ndarray:
        """Computes matrix M (N-1, 6) where N is the number of joint positions
            TI_matrix = [SUBTASK, FROM, TO, TI_VALUE, TI_TYPE]
        """
        # Get number of blocks to move
        NUMBER_OF_BLOCKS, _, _ = np.shape(self.ik_solution_tensor)

        # Add HOME to ORIGIN entries in TI_matrix
        # Get timing essential (TE) joint positions (JP)
        
        
        # Add all Task entries
        for block_number in range(NUMBER_OF_BLOCKS):
            # Evaluate cases
            
            # MISSING_BLOCK_TO_BLOCK
            if self.missing_block == block_number:
                pass
            
            # HOME_TO_BLOCK_TO_BLOCK
            elif block_number == 0:
                
                
                timing_essential_jps = self._get_timing_essential_positions(ACTION_PROCEDURES.HOME_TO_BLOCK_TO_BLOCK, block_number)
                print(timing_essential_jps)
                # Get TIs
                HOME_TO_BLOCK_TO_BLOCK_MOVES_TIs = self._get_duration_between_positions(timing_essential_jps)
                print(HOME_TO_BLOCK_TO_BLOCK_MOVES_TIs)


                # self.compute_timing_interval_by_procedure(ACTION_PROCEDURES.HOME_TO_BLOCK_TO_BLOCK, durations)
                # Combine
                # Add to TI_matrix
                # GRIPPER_OPEN = [0, *[None]*12, TI_CONSTANT_VALUES.FULLY_OPEN_GRIPPER, TI_TYPES.FULLY_OPEN_GRIPPER]
                # HOME_TO_BGP = [0, *HOME, *BGP, HOME_TO_ORIGIN_TIs[0], TI_TYPES.MOVE]
                # BGP_TO_GP = [0, *BGP, *GP, HOME_TO_ORIGIN_TIs[1], TI_TYPES.MOVE]
                
                # TI_matrix.append(GRIPPER_OPEN)
                # TI_matrix.append(HOME_TO_BGP)
                # TI_matrix.append(BGP_TO_GP)
            # BLOCK_TO_BLOCK_TO_HOME
            elif block_number == NUMBER_OF_BLOCKS -1:
                pass
            # BLOCK_TO_BLOCK
            else:
                pass


        # Add BTP to HOME entries in TI_matrix



        


if __name__ == "__main__":
    import yaml

    with open(f"../../config/tasks/2_blocks.yaml", "r") as file:
        task_config = yaml.safe_load(file)

    robot_model = UR3e()
    task_ik_solutions = robot_model.compute_ik_task_tensor(task_config)
    timing_model = TimingModel()
    timing_model.set_ik_solution_tensor(task_ik_solutions)
    timing_model.compute_timing_intervals()