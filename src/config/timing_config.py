from dataclasses import dataclass
import numpy as np

@dataclass
class ROBOT_PHYSICS:
    JOINT_SPEED_MAX_DEG = 60 # deg/s
    JOINT_ACCELERATION_DEG = 80 # deg/s^2
    JOINT_SPEED_MAX_RAD = np.deg2rad(JOINT_SPEED_MAX_DEG) # rad/s
    JOINT_ACCELERATION_RAD = np.deg2rad(JOINT_ACCELERATION_DEG) # rad/s^2
    ACCELERATION_TRAP_TIME = JOINT_SPEED_MAX_RAD / JOINT_ACCELERATION_RAD # Time it takes to reach maximun velocity
    ACCELERATION_DIST = 1/2 * JOINT_ACCELERATION_RAD * ACCELERATION_TRAP_TIME**2


@dataclass
class TI_TYPES:
    # ---- EXTERNAL ----
    GRIP = "GRIP"
    PARTLY_OPEN_GRIPPER = "PARTLY_OPEN_GRIPPER"
    FULLY_OPEN_GRIPPER = "FULLY_OPEN_GRIPPER"
    INITALIZATION_CODE_DELAY = "INITALIZATION_CODE_DELAY"    
    MOVE = "MOVE"    

    # ---- INTERNAL FOR CONVINENCE ----
    BGP_TO_GP = MOVE
    GP_TO_BGP = MOVE 
    BGP_TO_BTP = MOVE
    BTP_TO_TP = MOVE
    TP_TO_BTP = MOVE
    BTP_TO_BGP = MOVE
    HOME_TO_BGP = MOVE
    BTP_TO_HOME = MOVE
    BGP_TO_BGPnext = MOVE
    


@dataclass
class TI_CONSTANT_VALUES:
    GRIP_TI = 0.7
    PARTLY_OPEN_GRIPPER = 0.3
    FULLY_OPEN_GRIPPER = 1.1
    INITALIZATION_DELAY = 0.5
    ACCELERATION_TRAP = ROBOT_PHYSICS.ACCELERATION_TRAP_TIME


@dataclass
class ACTION_PROCEDURES:
    HOME_TO_BLOCK_TO_BLOCK = [TI_TYPES.FULLY_OPEN_GRIPPER, TI_TYPES.HOME_TO_BGP, TI_TYPES.BGP_TO_GP, TI_TYPES.GRIP, TI_TYPES.GP_TO_BGP, TI_TYPES.BGP_TO_BTP, TI_TYPES.BTP_TO_TP, TI_TYPES.PARTLY_OPEN_GRIPPER, TI_TYPES.TP_TO_BTP, TI_TYPES.FULLY_OPEN_GRIPPER, TI_TYPES.INITALIZATION_CODE_DELAY, TI_TYPES.BTP_TO_BGP]
    BLOCK_TO_BLOCK = [TI_TYPES.BGP_TO_GP, TI_TYPES.GRIP, TI_TYPES.GP_TO_BGP, TI_TYPES.BGP_TO_BTP, TI_TYPES.BTP_TO_TP, TI_TYPES.PARTLY_OPEN_GRIPPER, TI_TYPES.TP_TO_BTP, TI_TYPES.FULLY_OPEN_GRIPPER, TI_TYPES.INITALIZATION_CODE_DELAY, TI_TYPES.BTP_TO_BGP]
    BLOCK_TO_BLOCK_TO_HOME = [*BLOCK_TO_BLOCK[:-2], TI_TYPES.BTP_TO_HOME]
    MISSING_BLOCK_TO_BLOCK_TO_BLOCK = [TI_TYPES.INITALIZATION_CODE_DELAY, TI_TYPES.GP_TO_BGP, TI_TYPES.BGP_TO_BGPnext, *BLOCK_TO_BLOCK]
