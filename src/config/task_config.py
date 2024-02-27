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
    NO_BLOCKS = "blocks"  # number of blocks in the task

    ORIGIN = "origin"
    TARGET = "target"
    x = "x"
    y = "y"

    """Config for the task.
    To access waypoints, use block_config[block_number][WAYPOINTS],
    where block_number is the block number.
    To access grip percentage, use block_config[block_number][GRIP_PERCENTAGE],
    where block_number is the block number.
    """
    block_config_joint_positions = {
        NO_BLOCKS: 3,
        1: {
            WAYPOINTS: [
                [2.04, -1.81, -1.44, -1.41, 1.57, 0.54],
                [2.04, -1.85, -1.74, -1.099, 1.51, 0.55],
                [0.618, -1.72, -1.59, -1.32, 1.56, 0.27],
                [0.62, -1.88, -1.63, -1.24, 1.56, 0.23],
            ],
            GRIP_PERCENTAGE: 41,
            TIMING_THRESHOLD: 6,
        },
        2: {
            WAYPOINTS: [
                [2.82, -1.40, -1.91, -1.43, 1.58, 0.45],
                [2.82, -1.51, -2.14, -1.08, 1.59, 0.49],
                [0.63, -1.74, -1.46, -1.49, 1.56, 0.29],
                [0.59, -1.83, -1.46, -1.43, 1.61, 0.20],
            ],
            GRIP_PERCENTAGE: 41,
            TIMING_THRESHOLD: 11.4,
        },
        3: {
            WAYPOINTS: [
                [3.43, -1.88, -1.56, -1.22, 1.59, 1.04],
                [3.45, -1.97, -1.68, -1.02, 1.59, 1.07],
                [0.59, -1.87, -1.14, -1.74, 1.62, 0.32],
                [0.61, -1.78, -1.39, -1.53, 1.58, 0.29],
            ],
            GRIP_PERCENTAGE: 41,
            TIMING_THRESHOLD: 13,
        },
    }

    block_config_1_block = {
        NO_BLOCKS: 1,
        1: {
            ORIGIN: {
                x: 0,
                y: 0,
            },
            TARGET: {
                x: 12,
                y: -5,
            },
            TIMING_THRESHOLD: 6,
        },
    }

    block_config_square = {
        NO_BLOCKS: 12,
        1: {
            ORIGIN: {
                x: 4,
                y: 0,
            },
            TARGET: {
                x: 13,
                y: -5,
            },
            TIMING_THRESHOLD: 6,
        },
        2: {
            ORIGIN: {
                x: 5,
                y: 1,
            },
            TARGET: {
                x: 12,
                y: -5,
            },
            TIMING_THRESHOLD: 6,
        },
        3: {
            ORIGIN: {
                x: 6,
                y: 0,
            },
            TARGET: {
                x: 11,
                y: -5,
            },
            TIMING_THRESHOLD: 6,
        },
        4: {
            ORIGIN: {
                x: 7,
                y: 1,
            },
            TARGET: {
                x: 10,
                y: -5,
            },
            TIMING_THRESHOLD: 6,
        },
        5: {
            ORIGIN: {
                x: 8,
                y: 0,
            },
            TARGET: {
                x: 13,
                y: -4,
            },
            TIMING_THRESHOLD: 6,
        },
        6: {
            ORIGIN: {
                x: 9,
                y: 1,
            },
            TARGET: {
                x: 10,
                y: -4,
            },
            TIMING_THRESHOLD: 6,
        },
        7: {
            ORIGIN: {
                x: 10,
                y: 0,
            },
            TARGET: {
                x: 13,
                y: -3,
            },
            TIMING_THRESHOLD: 6,
        },
        8: {
            ORIGIN: {
                x: 11,
                y: 1,
            },
            TARGET: {
                x: 10,
                y: -3,
            },
            TIMING_THRESHOLD: 6,
        },
        9: {
            ORIGIN: {
                x: 12,
                y: 0,
            },
            TARGET: {
                x: 13,
                y: -2,
            },
            TIMING_THRESHOLD: 6,
        },
        10: {
            ORIGIN: {
                x: 13,
                y: 1,
            },
            TARGET: {
                x: 12,
                y: -2,
            },
            TIMING_THRESHOLD: 6,
        },
        11: {
            ORIGIN: {
                x: 14,
                y: 0,
            },
            TARGET: {
                x: 11,
                y: -2,
            },
            TIMING_THRESHOLD: 6,
        },
        12: {
            ORIGIN: {
                x: 8,
                y: 2,
            },
            TARGET: {
                x: 10,
                y: -2,
            },
            TIMING_THRESHOLD: 6,
        },
    }
