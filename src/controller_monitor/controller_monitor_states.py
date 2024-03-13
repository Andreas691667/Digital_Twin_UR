class CM_STATES:
    """Class for the states of the ControllerMonitor"""
    INITIALIZING = 0 # Before establishing connection to broker
    WAITING_FOR_TASK_VALIDATION = 1 # Waiting for task validation
    WAITING_FOR_USER_INPUT = 2 # Waits for user to press enter
    NORMAL_OPERATION = 3 # Performing task
    WAITING_FOR_FAULT_RESOLUTION = 4 # Waiting for resolution
    MANUEL_INTERVENTION = 5 # Waiting for manuel intervention
    SHUTTING_DOWN = 6 # Shutting down