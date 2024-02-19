import numpy as np


class TASK_CONFIG:
    TARGET_POSE = "TARGET_POSE"
    JOINT_POSITIONS = "JOINT_POSITIONS"
    STOP = "STOP"  # Decelerates to zero with deceleration given by argument
    ARG_TYPES = [TARGET_POSE, JOINT_POSITIONS, STOP]

    ur_script_mappings = {TARGET_POSE: "movel", JOINT_POSITIONS: "movej", STOP: "stopj"}

    # matrix of 4 waypoints and grip percentage
    WAYPOINTS = "waypoints"
    GRIP_PERCENTAGE = "grip_percentage"
    TIMING_THRESHOLD = "timing_threshold"
    BLOCKS = 2  # number of blocks in the task

    """Config for the task.
    To access waypoints, use block_config[block_number][WAYPOINTS],
    where block_number is the block number.
    To access grip percentage, use block_config[block_number][GRIP_PERCENTAGE],
    where block_number is the block number."""
    block_config = {
        1: {
            WAYPOINTS: np.array(
                [
                    [2.04, -1.81, -1.44, -1.41, 1.57, 0.54],
                    [2.04, -1.85, -1.74, -1.099, 1.51, 0.55],
                    [0.618, -1.72, -1.59, -1.32, 1.56, 0.27],
                    [0.62, -1.88, -1.63, -1.24, 1.56, 0.23],
                ]
            ),
            GRIP_PERCENTAGE: 41,
            TIMING_THRESHOLD: 3,
        },
        2: {
            WAYPOINTS: np.array(
                [
                    [2.82, -1.40, -1.91, -1.43, 1.58, 0.45],
                    [2.82, -1.51, -2.14, -1.08, 1.59, 0.49],
                    [0.63, -1.74, -1.46, -1.49, 1.56, 0.29],
                    [0.59, -1.83, -1.46, -1.43, 1.61, 0.20],
                ]
            ),
            GRIP_PERCENTAGE: 41,
            TIMING_THRESHOLD: 0.5,
        },
    }
