from sys import path
path.append("../..")

from ur3e.ur3e import UR3e
import numpy as np
from config.grid_config import GRID_CONFIG
from dt_services.TimingModel import TimingModel
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
    

class TaskTrajectoryTimingEstimator(TimingModel):
    def format_task_with_timings (self, timing_essential_joint_positions: np.ndarray, speed_profiles_and_delays: np.ndarray) -> np.ndarray:
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
        values = self.strl2floatl(speed_profiles_and_delays[0, :])

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
    
    
    def get_task_trajectory_timings (self, task_config):
            """Computes the thresholds corresponding to block movements
            see compute_ik_solutions for input format
            """
            ik_solutions = self.compute_ik_solutions(task_config)
            number_of_blocks: int = task_config[GRID_CONFIG.NO_BLOCKS] 
            
            # Initialize threshold
            task_with_timings = np.array([])
        
            # Calculate thresholds for blocks, if there is more than one block to move
            for block_number in range(number_of_blocks+1):
                # Get positions which matters for the timing calculations
                # That is: All subtasks jp of the first task, and the first subtask jp of the next task
                timing_essential_positions = self.get_timing_essential_positions(ik_solutions, block_number, number_of_blocks, -1)
                
                # Get speed profile timing intervals
                # TODO: Agree on this termonology througtout codebase!
                _, speed_profiles_ti_extended, speed_profiles_ti = self.get_duration_between_positions(timing_essential_positions, block_number)
                
                # Add additional ties
                _, speed_profiles_ti_with_delays = self.add_additional_ti(speed_profiles_ti_extended, speed_profiles_ti, block_number, number_of_blocks, -1)

                subtask_with_timings = self.format_task_with_timings(timing_essential_positions, speed_profiles_ti_with_delays)

                # Add to task with timings
                if (len(task_with_timings) == 0):
                    task_with_timings = subtask_with_timings
                else:
                    task_with_timings = np.vstack((task_with_timings, subtask_with_timings)) 
                   
            return task_with_timings
    
    
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
        elif block_number == 0:
            home_sol = self.get_home_ik_sol()
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
            HOME = self.get_home_ik_sol()
            timing_essential_positions = np.vstack((GP, BGP, BTP, TP, BTP, HOME))
        
        return timing_essential_positions


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



