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
            WAYPOINTS: [
                [0.5, 0.5, 0.5, 0.5],
                [0.5, 0.5, 0.5, 0.5],
                [0.5, 0.5, 0.5, 0.5],
                [0.5, 0.5, 0.5, 0.5],
            ],
            GRIP_PERCENTAGE: 41,
        },
        2 : {
            WAYPOINTS: [
                [0.5, 0.5, 0.5, 0.5],
                [0.5, 0.5, 0.5, 0.5],
                [0.5, 0.5, 0.5, 0.5],
                [0.5, 0.5, 0.5, 0.5],
            ],
            GRIP_PERCENTAGE: 41,
        },
    }
