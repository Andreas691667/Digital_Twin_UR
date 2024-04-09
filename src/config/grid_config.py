class GRID_CONFIG:
    # ----- NOT USED -----
    # TARGET_POSE = "TARGET_POSE"
    # JOINT_POSITIONS = "JOINT_POSITIONS"
    # STOP = "STOP"  # Decelerates to zero with deceleration given by argument
    # ARG_TYPES = [TARGET_POSE, JOINT_POSITIONS, STOP]


    # ----- TASK CONFIGURATION -----
    NO_BLOCKS = "NO_BLOCKS"  # number of blocks in the task
    # ----- END TASK CONFIGURATION -----


    # ----- SUBTASK CONFIGURATION -----
    # Origin and target coordinates
    ORIGIN = "ORIGIN"
    TARGET = "TARGET"
    x = "x"
    y = "y"

    # Additional parameters
    TIMING_THRESHOLD = "TIMING_THRESHOLD"
    ROTATE_WRIST = "ROTATE_WRIST"
    # ----- END SUBTASK CONFIGURATION -----

    # ----- GRID PARAMETERS -----
    # Parameter names for dictionary
    Z_BASE_MIN = "Z_BASE_MIN"
    HOLE_DIST = "HOLE_DIST"
    Z_BASE_MIN = "Z_BASE_MIN"
    X_BASE_MAX = "X_BASE_MAX"
    X_BASE_MIN = "X_BASE_MIN"
    Y_BASE_MAX = "Y_BASE_MAX"
    Y_BASE_MIN = "Y_BASE_MIN"
    GRIP_Z = "GRIP_Z"
    BEFORE_GRIP_Z = "BEFORE_GRIP_Z"

    # Parameter values
    GRID_PARAMETERS = {
        HOLE_DIST: 0.04,
        Z_BASE_MIN: 0.173,
        X_BASE_MAX: 0.152,
        X_BASE_MIN: -0.12051,
        Y_BASE_MAX: 0.4729,
        Y_BASE_MIN: 0.28071,
    }

    # place size (in grid points)
    PLACE_WIDTH = 5
    PLACE_HEIGHT = 10

    GRID_COORDINATES = {
        GRIP_Z: 0.3,
        BEFORE_GRIP_Z: 1.55
    }

    # gripper widths
    GRIPPER_WIDTH_TARGET = 0.06 # in meters
    GRIPPER_WIDTH_ORIGIN = 0.1 # in meters

    # max block width
    BLOCK_WIDTH = 0.035 # in meters

    # ----- END GRID PARAMETERS -----
    
    # MITIGATION STRATEGIES
    class MITIGATION_STRATEGIES:
        SHIFT_ORIGIN = "SHIFT_ORIGIN"
        TRY_PICK_STOCK = "TRY_PICK_STOCK"
    
    # COORDINATES OF PICK_STOCK
    PICK_STOCK_COORDINATES = {
        1: {
            ORIGIN: {
                x: -3,
                y: 1
            },
            TIMING_THRESHOLD: 5,
        },
        
        2: {
            ORIGIN: {
                x: -5,
                y: 1
            },
            TIMING_THRESHOLD: 5,
        },
        
        3: {
            ORIGIN: {
                x: -4,
                y: 2
            },
            TIMING_THRESHOLD: 4,
        },
        
        4: {
            ORIGIN: {
                x: -3,
                y: 3
            },
            TIMING_THRESHOLD: 20,
        },
    }



    """Config for the task.
    To access waypoints, use block_config[block_number][WAYPOINTS],
    where block_number is the block number.
    To access grip percentage, use block_config[block_number][GRIP_PERCENTAGE],
    where block_number is the block number.
    """
    DYNAMIC_THRESHOLD_LONG_TASK = {
        NO_BLOCKS: 1,
        0: {
            ORIGIN: {
                x: -5,
                y: 1,
                ROTATE_WRIST: False,
            },
            TARGET: {
                x: 14,
                y: -8,
                ROTATE_WRIST: False,
            },
            TIMING_THRESHOLD: 50,
        },
    }
    
    DYNAMIC_THRESHOLD_SMALL_TASK = {
        NO_BLOCKS: 1,
        0: {
            ORIGIN: {
                x: 1,
                y: 3,
                ROTATE_WRIST: False,
            },
            TARGET: {
                x: 3,
                y: 3,
                ROTATE_WRIST: False,
            },
            TIMING_THRESHOLD: 50,
        },
    }

    DYNAMIC_THRESHOLD_TWO_BLOCKS = {
        NO_BLOCKS: 2,
        0: {
            ORIGIN: {
                x: 0,
                y: 0,
                ROTATE_WRIST: False,
            },
            TARGET: {
                x: 13,
                y: -5,
                ROTATE_WRIST: False,
            },
            TIMING_THRESHOLD: 50,
        },
        
        1: {
            ORIGIN: {
                x: 0,
                y: 2,
                ROTATE_WRIST: False,
            },
            TARGET: {
                x: 12,
                y: -5,
                ROTATE_WRIST: False,
            },
            TIMING_THRESHOLD: 50,
        },
    }

    HOME_POSITION = {
        x : 11,
        y : 2
    }

    block_config_heart = {
        NO_BLOCKS: 14,
        0: {
            ORIGIN: {
                x: -1,
                y: 1,
                ROTATE_WRIST: False,
            },
            TARGET: {
                x: 13,
                y: -4,
                ROTATE_WRIST: False,
            },
            TIMING_THRESHOLD: 20,
        },
        1: {
            ORIGIN: {
                x: 1,
                y: 1,
                ROTATE_WRIST: False,
            },
            TARGET: {
                x: 12,
                y: -5,
                ROTATE_WRIST: False,
            },
            TIMING_THRESHOLD: 20, #works
        },
        2: {
            ORIGIN: {
                x: 3,
                y: 1,
                ROTATE_WRIST: False,
            },
            TARGET: {
                x: 11,
                y: -6,
                ROTATE_WRIST: False,
            },
            TIMING_THRESHOLD: 20, #works
        },
        3: {
            ORIGIN: {
                x: 5,
                y: 1,
                ROTATE_WRIST: False,
            },
            TARGET: {
                x: 10,
                y: -7,
                ROTATE_WRIST: False,
            },
            TIMING_THRESHOLD: 20, #works
        },
        4: {
            ORIGIN: {
                x: 7,
                y: 1,
                ROTATE_WRIST: False,
            },
            TARGET: {
                x: 10,
                y: -8.5,
                ROTATE_WRIST: False,
            },
            TIMING_THRESHOLD: 20, # works
        },
        5: {
            ORIGIN: {
                x: -1,
                y: 3,
                ROTATE_WRIST: False,
            },
            TARGET: {
                x: 11,
                y: -9,
                ROTATE_WRIST: False,
            },
            TIMING_THRESHOLD: 20,
        },
        6: {
            ORIGIN: {
                x: 1,
                y: 3,
                ROTATE_WRIST: False,
            },
            TARGET: {
                x: 12,
                y: -8,
                ROTATE_WRIST: False,
            },
            TIMING_THRESHOLD: 20,
        },
        7: {
            ORIGIN: {
                x: 3,
                y: 3,
                ROTATE_WRIST: False,
            },
            TARGET: {
                x: 13,
                y: -7,
                ROTATE_WRIST: False,
            },
            TIMING_THRESHOLD: 20,
        },
        8: {
            ORIGIN: {
                x: 5,
                y: 3,
                ROTATE_WRIST: False,
            },
            TARGET: {
                x: 14,
                y: -8,
                ROTATE_WRIST: False,
            },
            TIMING_THRESHOLD: 20,
        },
        9: {
            ORIGIN: {
                x: 7,
                y: 3,
                ROTATE_WRIST: False,
            },
            TARGET: {
                x: 15,
                y: -9,
                ROTATE_WRIST: False,
            },
            TIMING_THRESHOLD: 20,
        },
        10: {
            ORIGIN: {
                x: -1,
                y: 5,
                ROTATE_WRIST: False,
            },
            TARGET: {
                x: 16,
                y: -8.5,
                ROTATE_WRIST: False,
            },
            TIMING_THRESHOLD: 20,
        },
        11: {
            ORIGIN: {
                x: 1,
                y: 5,
                ROTATE_WRIST: False,
            },
            TARGET: {
                x: 16,
                y: -7,
                ROTATE_WRIST: False,
            },
            TIMING_THRESHOLD: 20,
        },
        
        12: {
            ORIGIN: {
                x: 3,
                y: 5,
                ROTATE_WRIST: False,
            },
            TARGET: {
                x: 15,
                y: -6,
                ROTATE_WRIST: False,
            },
            TIMING_THRESHOLD: 20,
        },        
        13: {
            ORIGIN: {
                x: 5,
                y: 5,
                ROTATE_WRIST: False,
            },
            TARGET: {
                x: 14,
                y: -5,
                ROTATE_WRIST: False,
            },
            TIMING_THRESHOLD: 20,
        },
    }
