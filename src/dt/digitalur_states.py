class DT_STATES:
    """Class for the states of the digital twin"""
    INITIALIZING = 0
    WAITING_TO_RECEIVE_TASK = 1
    WAITING_FOR_TASK_TO_START = 2
    MONITORING_PT = 3
    FAULT_RESOLUTION = 4
