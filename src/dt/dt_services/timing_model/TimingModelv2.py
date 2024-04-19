# Imports
import numpy as np
from sys import path
path.append("../..")
from ur3e.ur3e import UR3e # for testing!
from config.timing_config import ROBOT_PHYSICS
from config.timing_config import TIs
TI_TYPES = TIs.TYPES
TI_CONSTANT_VALUES = TIs.VALUES
from config.timing_config import TI_SEQUENCES
TI_SEQUENCE_TYPES = TI_SEQUENCES.TYPES
TI_SEQUENCE_VALUES = TI_SEQUENCES.VALUES

# TimingModel
class TimingModel:
    def __init__(self, robot_model) -> None:
        # TI_matrix = [SUBTASK, FROM, TO, TI_VALUE, TI_TYPE], TI: "Timing Interval"
        # SUBTASK: The move of a block from its origin to a target
        # FROM: The joint position to move from
        # TO: The joint position to move to
        # TI_VALUE: "Timing Interval Value"
        # TI_TYPE: "Timing Interval Type"
        self.TI_matrix = []

        # Other attributes
        self.robot_model = robot_model
        self.timing_essential_joint_positions = []
        self.move_TIs = []
        self.from_to_matrix = []
        self.ik_solution_tensor = []
        self.number_of_blocks = -1
    
    
    # ---- INTERNAL METHODS ----
    def _get_distances_of_leading_axes(self) -> np.ndarray:
        """
        Get sliding windows differences
        If input is NxM, then output is (N-1)xM and the leading axis of N-1 x 1
        """
        # sliding window differences
        joint_positions_differences = np.abs(self.from_to_matrix[:, :6] - self.from_to_matrix[:, 6:])
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
        JOINT_SPEED_MAX_RAD = ROBOT_PHYSICS.JOINT_SPEED_MAX_RAD
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


    def _compute_move_TIs_from_TEs (self):
        """
        Input is joins_positions of NxM, where N are the number of positions and M are DOF
        Outputs the combined duration between each joint position and the individual durations
        """ 
        # Get the distances of the leading axis
        distances_of_leading_axis = self._get_distances_of_leading_axes()
        
        # Get timing intervals and speedprofiles/delays + descriptions
        durations = self._get_durations_of_leading_axis(distances_of_leading_axis) 

        self.move_TIs = durations


    def _compute_timing_essential_positions (self, TI_sequence_type, block_number):
        """Get timing essential joint positions"""
        # Initialize output
        timing_essential_positions = []

        # Retrieve common variables
        joint_positions_current_block = self.ik_solution_tensor[block_number, :, :]
        HOME = self.robot_model.get_home_ik_solution()
        BGP = joint_positions_current_block[0, :]
        GP = joint_positions_current_block[1, :] 
        BTP = joint_positions_current_block[2, :] 
        TP = joint_positions_current_block[3, :]

        joint_positions_next_block = []
        BGP_next = []
        GP_next = []

        if block_number != self.number_of_blocks - 1:
            joint_positions_next_block = self.ik_solution_tensor[block_number+1, :, :]
            BGP_next = joint_positions_next_block[0, :]
            GP_next = joint_positions_next_block[1, :]

        
        # First block (or last if there are only one)
        if TI_sequence_type == TI_SEQUENCE_TYPES.HOME_TO_BLOCK_TO_BLOCK:
            timing_essential_positions = np.vstack((HOME, BGP, GP, BGP, BTP, TP, BTP, BGP_next, GP_next))
        
        # Middle block
        elif TI_sequence_type == TI_SEQUENCE_TYPES.BLOCK_TO_BLOCK:
            timing_essential_positions = np.vstack((BGP, GP, BGP, BTP, TP, BTP, BGP_next, GP_next))

        # Last move
        else:
            timing_essential_positions = np.vstack((BGP, GP, BGP, BTP, TP, BTP, HOME))
        
        # Set attribute
        self.timing_essential_joint_positions = timing_essential_positions
        self.from_to_matrix = np.hstack((timing_essential_positions[:-1,:], timing_essential_positions[1:, :]))
    
    def _is_move_TI (self, TI) -> bool:
        """Determines if it is a move timing interval"""
        if TI == TIs.TYPES.MOVE_WITH_OBJECT or TI == TIs.TYPES.MOVE_WITHOUT_OBJECT:
            return True
        else: return False

    def _compute_TIs_by_TI_sequence_and_move_TIs(self, TI_sequence_type, block_number):
        """Append to TIs to TI_matrix"""
        move_counter = 0
        TI_SEQUENCE_VALUE = TI_SEQUENCE_VALUES[TI_sequence_type]
        for _, TI_TYPE in enumerate(TI_SEQUENCE_VALUE):
            if self._is_move_TI(TI_TYPE):
                entry = [block_number, *self.from_to_matrix[move_counter, :], self.move_TIs[move_counter], TI_TYPE]
                self.TI_matrix.append(entry)
                move_counter += 1
            else:
                entry = [block_number, *[None]*12, TI_CONSTANT_VALUES[TI_TYPE], TI_TYPE]
                self.TI_matrix.append(entry)

    def _block_number_to_TI_sequence_type(self, block_number, initializing=True) -> list:
        """Maps a block_number to a timing interval sequence""" 
        # HOME_TO_BLOCK_TO_BLOCK
        if block_number == 0 and initializing:
                return TI_SEQUENCE_TYPES.HOME_TO_BLOCK_TO_BLOCK
            
        # BLOCK_TO_BLOCK_TO_HOME
        elif block_number == self.number_of_blocks - 1:
            return TI_SEQUENCE_TYPES.BLOCK_TO_BLOCK_TO_HOME
        
        # BLOCK_TO_BLOCK
        else:
            return TI_SEQUENCE_TYPES.BLOCK_TO_BLOCK
        
    # ---- PUBLIC METHODS ----
    def get_duration_between_positions (self, joint_positions: np.ndarray) -> np.ndarray:
        """Get durations between joint positions"""
        # Set from_to_matrix
        self.from_to_matrix = np.hstack((joint_positions[:-1,:], joint_positions[1:, :]))
        
        # Get the distances of the leading axis
        distances_of_leading_axis = self._get_distances_of_leading_axes()
        
        # Get timing intervals and speedprofiles/delays + descriptions
        durations = self._get_durations_of_leading_axis(distances_of_leading_axis) 

        return durations
    
    def set_ik_solution_tensor (self, ik_solution_tensor):
        """Sets ik solutions tensor"""
        self.ik_solution_tensor = ik_solution_tensor
        self.number_of_blocks, _, _ = np.shape(ik_solution_tensor)
    
    def get_TI_matrix (self) -> list[list]:
        """Returns TI matrix"""
        return self.TI_matrix
    
    def compute_timing_intervals (self, start_block=0, initializing=True) -> np.ndarray:
        """Computes matrix M (N-1, 6) where N is the number of joint positions
            TI_matrix = [SUBTASK, FROM, TO, TI_VALUE, TI_TYPE]
        """
        # Get number of blocks to move
        NUMBER_OF_BLOCKS = self.number_of_blocks 
        
        # Add all Task entries
        for block_number in range(start_block, NUMBER_OF_BLOCKS):
            # Get TI_sequence
            TI_sequence_type = self._block_number_to_TI_sequence_type(block_number, initializing)

            # Get Timing Essential Joint Positions
            self._compute_timing_essential_positions(TI_sequence_type, block_number)
            
            # Get move TIs
            self._compute_move_TIs_from_TEs()
            
            # Compute all TIs from TI Sequence
            self._compute_TIs_by_TI_sequence_and_move_TIs(TI_sequence_type, block_number)

if __name__ == "__main__":
    import yaml

    with open(f"../../config/tasks/2_blocks.yaml", "r") as file:
        task_config = yaml.safe_load(file)

    robot_model = UR3e()
    task_ik_solutions = robot_model.compute_ik_task_tensor(task_config)
    timing_model = TimingModel(robot_model)
    timing_model.set_ik_solution_tensor(task_ik_solutions)
    timing_model.compute_timing_intervals(1)
    print(timing_model.TI_matrix)