class DT_STATES:
    """Class for the states of the digital twin"""
    INITIALIZING = 0
    WAITING_TO_RECEIVE_TASK = 1
    VALIDATING_TASK = 2
    WAITING_FOR_TASK_TO_START = 3
    NORMAL_OPERATION = 4
    FAULT_RESOLUTION = 5
