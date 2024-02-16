class TASK_CONFIG:
    TARGET_POSE = "TARGET_POSE"
    JOINT_POSITIONS = "JOINT_POSITIONS"
    STOP = "STOP"  # Decelerates to zero with deceleration given by argument
    ARG_TYPES = [TARGET_POSE, JOINT_POSITIONS, STOP]

    ur_script_mappings = {TARGET_POSE: "movel", JOINT_POSITIONS: "movej", STOP: "stopj"}

    # matrix of 4 waypoints and grip percentage
    WAYPOINTS = "waypoints"
    GRIP_PERCENTAGE = "grip_percentage"

    """Config for the task.
    To access waypoints, use block_config[block_number][WAYPOINTS],
    where block_number is the block number.
    To access grip percentage, use block_config[block_number][GRIP_PERCENTAGE],
    where block_number is the block number."""
    block_config = {
        1 : {
            WAYPOINTS:
                [2.04, -1.81, -1.44, -1.41, 1.57, 0.54,
                  2.04, -1.85, -1.74, -1.099, 1.51, 0.55,
                  0.618, -1.72, -1.59, -1.32, 1.56, 0.27,
                  0.62, -1.88, -1.63, -1.24, 1.56, 0.23]
            ,
            GRIP_PERCENTAGE: 41,
        }
    }
