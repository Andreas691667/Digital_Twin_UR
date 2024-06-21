from dataclasses import dataclass
import numpy as np

@dataclass
class ROBOT_PHYSICS:
    JOINT_SPEED_MAX_DEG = 60 # deg/s
    JOINT_ACCELERATION_DEG = 80 # deg/s^2
    JOINT_SPEED_MAX_RAD = np.deg2rad(JOINT_SPEED_MAX_DEG) # rad/s
    JOINT_ACCELERATION_RAD = np.deg2rad(JOINT_ACCELERATION_DEG) # rad/s^2
    ACCELERATION_TRAP_TIME = JOINT_SPEED_MAX_RAD / JOINT_ACCELERATION_RAD   # Time it takes to reach maximun velocity
    ACCELERATION_DIST = 1/2 * JOINT_ACCELERATION_RAD * ACCELERATION_TRAP_TIME**2
    ADJUSTMENT_EPSILON_JOINT_0_TO_4 = 0.07 # Epsilon for adjusting the timing interval
    ADJUSTMENT_EPSILON_JOINT_5 = 0.18 # Epsilon for adjusting the timing interval


@dataclass
class TIs:
    # ---- EXTERNAL ----
    @dataclass
    class TYPES:
        GRIP = "GRIP"
        PARTLY_OPEN_GRIPPER = "PARTLY_OPEN_GRIPPER"
        FULLY_OPEN_GRIPPER = "FULLY_OPEN_GRIPPER"
        INITALIZATION_CODE_DELAY = "INITALIZATION_CODE_DELAY"    
        MOVE_WITH_OBJECT = "MOVE_WITH_OBJECT"
        MOVE_WITHOUT_OBJECT = "MOVE_WITHOUT_OBJECT" 
        START_DELAY = "START_DELAY"
        HOME_DELAY = "HOME_DELAY"

        # ---- INTERNAL FOR CONVINENCE ----
        BGP_TO_GP = MOVE_WITHOUT_OBJECT
        GP_TO_BGP = MOVE_WITH_OBJECT 
        BGP_TO_BTP = MOVE_WITH_OBJECT
        BTP_TO_TP = MOVE_WITH_OBJECT
        TP_TO_BTP = MOVE_WITHOUT_OBJECT
        BTP_TO_BGP = MOVE_WITHOUT_OBJECT
        HOME_TO_BGP = MOVE_WITHOUT_OBJECT
        BTP_TO_HOME = MOVE_WITHOUT_OBJECT
        BGP_TO_BGPnext = MOVE_WITHOUT_OBJECT
    
    VALUES = {
        TYPES.GRIP: 0.7,
        TYPES.PARTLY_OPEN_GRIPPER: 0.3,
        TYPES.FULLY_OPEN_GRIPPER: 1.1,
        TYPES.INITALIZATION_CODE_DELAY: 0.5,
        TYPES.START_DELAY: 1.1-0.5,
        TYPES.HOME_DELAY: 0.8
    }

@dataclass
class TI_SEQUENCES:
    # The Timing Interval sequence is the sequence of timing intervals associated with the timing interval cases.
    # The cases are determined from the block number, total number of blocks in the task and if there is a missing block
    # Sequence i have all TIs associated to moving block i
    # If we do not start from HOME and there is no missing block the sequence always start from BGP_TO_GP
    # If we do not end at HOME the sequence always end with INITIALIZATION_CODE_DELAY and BTP_TO_BGP
    
    @dataclass
    class TYPES:
        HOME_TO_BLOCK_TO_BLOCK = "HOME_TO_BLOCK_TO_BLOCK"
        BLOCK_TO_BLOCK = "BLOCK_TO_BLOCK"
        BLOCK_TO_BLOCK_TO_HOME = "BLOCK_TO_BLOCK_TO_HOME"
    
    _block_to_block = [TIs.TYPES.BGP_TO_GP, TIs.TYPES.GRIP, TIs.TYPES.GP_TO_BGP, 
                       TIs.TYPES.BGP_TO_BTP, TIs.TYPES.BTP_TO_TP, TIs.TYPES.PARTLY_OPEN_GRIPPER, 
                       TIs.TYPES.TP_TO_BTP, TIs.TYPES.FULLY_OPEN_GRIPPER, TIs.TYPES.INITALIZATION_CODE_DELAY, TIs.TYPES.BTP_TO_BGP]
    VALUES = {
        TYPES.HOME_TO_BLOCK_TO_BLOCK: [TIs.TYPES.START_DELAY, TIs.TYPES.HOME_TO_BGP, *_block_to_block],
        TYPES.BLOCK_TO_BLOCK: _block_to_block,
        TYPES.BLOCK_TO_BLOCK_TO_HOME: [*_block_to_block[:-3], TIs.TYPES.HOME_DELAY, TIs.TYPES.BTP_TO_HOME],
    }

if __name__ == "__main__":
    print(TI_SEQUENCES.VALUES[TI_SEQUENCES.TYPES.BLOCK_TO_BLOCK_TO_HOME])